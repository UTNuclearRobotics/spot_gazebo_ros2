/*
Copyright (c) 2019-2020, Juan Miguel Jimeno

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_controller.h>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

QuadrupedController::QuadrupedController():
    Node("quadruped_controller_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(clock_.now())),
    kinematics_(base_)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string knee_orientation;
    std::string urdf = "";

    double loop_rate = 200.0;

    // Per-joint rate cap for published targets, rad/s. Sized to sit well above
    // normal gait and only catch discontinuities: a 0.25 s swing traversing
    // ~1 rad is ~4 rad/s, so 20 rad/s leaves ~5x headroom and never shapes a
    // healthy step. Lower it if resumes still land hard; raise it if fast
    // swings visibly lag their targets.
    max_joint_velocity_ = 20.0;

    // Longest command gap that is honoured when sizing the ramp. Roughly one
    // swing period: past that the gait is already broken and stretching the
    // ramp further just makes the leg crawl while the phase races ahead.
    max_command_interval_ = 0.25;

    // Seconds of sim time between command-interval reports. 0 disables.
    timing_report_period_ = 5.0;

    // Lookahead horizon in seconds, and how many points to spread across it.
    // 0.15 s comfortably covers the ~130 ms worst-case command gap measured
    // under load. Points are spaced by doubling from 2*loop_period_, so they
    // are dense where accuracy matters and sparse out at the horizon.
    lookahead_horizon_ = 0.15;
    lookahead_points_ = 5;
    lookahead_count_ = 0;

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("gait.knee_orientation",       knee_orientation);
    this->get_parameter("publish_foot_contacts",       publish_foot_contacts_);
    this->get_parameter("publish_joint_states",        publish_joint_states_);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("gazebo",                      in_gazebo_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("max_joint_velocity",          max_joint_velocity_);
    this->get_parameter("max_command_interval",        max_command_interval_);
    this->get_parameter("timing_report_period",        timing_report_period_);
    this->get_parameter("lookahead_horizon",           lookahead_horizon_);
    this->get_parameter("lookahead_points",            lookahead_points_);
    this->get_parameter("urdf",                        urdf);
    
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/smooth", 10, std::bind(&QuadrupedController::cmdVelCallback_, this,  std::placeholders::_1));
    cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "body_pose", 1,  std::bind(&QuadrupedController::cmdPoseCallback_, this,  std::placeholders::_1));
    
    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if(publish_foot_contacts_ && !in_gazebo_)
    {
        foot_contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
    }

    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromFile(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());

    loop_period_ = 1.0 / loop_rate;
    last_valid_joints_.fill(std::numeric_limits<float>::quiet_NaN());
    last_published_joints_.fill(std::numeric_limits<float>::quiet_NaN());

    // Seeded so these carry the node's clock source (sim time). Subtracting
    // times from different sources throws, and last_publish_time_ is not read
    // until the first publish overwrites it anyway.
    last_publish_time_ = clock_.now();
    next_timing_report_ =
        last_publish_time_ + rclcpp::Duration::from_seconds(timing_report_period_);
    interval_sum_ = 0.0;
    interval_worst_ = 0.0;
    interval_count_ = 0;
    interval_late_ = 0;

    // Drive the control loop off the node clock (sim time when use_sim_time
    // is set) instead of a wall timer. The gait phase generator samples sim
    // time, so a wall timer decouples command emission from gait phase
    // whenever the real-time factor dips or jitters, producing leg stutter.
    loop_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration::from_seconds(loop_period_),
        std::bind(&QuadrupedController::controlLoop_, this));
    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_()
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4];

    // Seed with NaN so an aborted solve is DETECTABLE: Kinematics::inverse
    // returns early without writing the output array when ANY foot target is
    // unreachable (it aborts all 12 joints, not just the failing leg). The
    // previous seeding with last_valid_joints_ made that abort invisible —
    // the stale pose was silently re-published while the gait phase kept
    // advancing, so legs froze mid-gait (worst while turning, when foot
    // targets are pushed toward max extension) and then jumped on recovery.
    for(size_t i = 0; i < 12; i++)
    {
        target_joint_positions[i] = std::numeric_limits<float>::quiet_NaN();
    }

    const rclcpp::Time now = clock_.now();

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(now));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    bool valid = true;
    for(size_t i = 0; i < 12; i++)
    {
        if(std::isnan(target_joint_positions[i]))
        {
            valid = false;
            break;
        }
    }

    if(!valid)
    {
        bool have_last_valid = !std::isnan(last_valid_joints_[0]);
        if(!have_last_valid)
        {
            // No valid command yet (IK failed before any successful solve) —
            // publishing would command NaN/garbage. Skip this cycle.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "IK produced no valid joint targets; skipping joint command this cycle");
            publishFootContacts_(foot_contacts);
            return;
        }

        // Unreachable foot target mid-gait: hold the last valid pose (same
        // physical behavior as before) but say so — if this fires while
        // turning, the commanded velocity/height combination is exceeding
        // leg reach and should be reduced.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IK unreachable foot target (cmd vx=%.2f vy=%.2f wz=%.2f); "
            "holding last valid joint command",
            req_vel_.linear.x, req_vel_.linear.y, req_vel_.angular.z);
        for(size_t i = 0; i < 12; i++)
        {
            target_joint_positions[i] = last_valid_joints_[i];
        }
        // No lookahead while holding: the gait we would extrapolate is exactly
        // the gait that just failed to solve.
        lookahead_count_ = 0;
        publishFootContacts_(foot_contacts);
        publishJoints_(target_joint_positions);
        return;
    }

    for(size_t i = 0; i < 12; i++)
    {
        last_valid_joints_[i] = target_joint_positions[i];
    }

    buildLookahead_(now, target_joint_positions);

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
}

void QuadrupedController::buildLookahead_(const rclcpp::Time &now, const float current_joints[12])
{
    lookahead_count_ = 0;

    if(lookahead_points_ <= 0 || lookahead_horizon_ <= loop_period_)
    {
        return;
    }

    // Snapshot every piece of gait state velocityCommand() mutates before
    // evaluating any future phase, and restore it before returning. Skipping
    // this does not merely perturb the gait, it breaks it: run() drags
    // last_touchdown_ toward whatever time it is handed, and once that sits in
    // the future the next real cycle evaluates (time - last_touchdown_) on an
    // unsigned type and gets ~2^64 microseconds.
    const champ::LegController::GaitState saved = leg_controller_.saveGaitState();

    double offset = 2.0 * loop_period_;

    for(int k = 0; k < lookahead_points_ && lookahead_count_ < kMaxLookaheadPoints; k++)
    {
        if(offset > lookahead_horizon_)
        {
            offset = lookahead_horizon_;
        }

        geometry::Transformation future_foot_positions[4];
        float future_joints[12];
        for(size_t i = 0; i < 12; i++)
        {
            future_joints[i] = std::numeric_limits<float>::quiet_NaN();
        }

        const rclcpp::Time future = now + rclcpp::Duration::from_seconds(offset);

        body_controller_.poseCommand(future_foot_positions, req_pose_);
        leg_controller_.velocityCommand(future_foot_positions, req_vel_,
                                        rosTimeToChampTime(future));
        kinematics_.inverse(future_joints, future_foot_positions);

        bool usable = true;
        for(size_t i = 0; i < 12; i++)
        {
            if(std::isnan(future_joints[i]))
            {
                usable = false;
                break;
            }
        }

        // Plausibility guard, and it is doing real work rather than being
        // defensive padding. Kinematics::inverse leaves upper_leg_joint and
        // lower_leg_joint UNWRITTEN when it takes the reachability early
        // return, so a "solved" future point can carry stale stack values that
        // are not NaN and read as ordinary angles. Any sample that would need
        // more than max_joint_velocity_ to reach is not a real gait sample,
        // whatever it looks like.
        if(usable)
        {
            const double budget = max_joint_velocity_ * offset;
            for(size_t i = 0; i < 12; i++)
            {
                if(std::fabs(static_cast<double>(future_joints[i] - current_joints[i])) > budget)
                {
                    usable = false;
                    break;
                }
            }
        }

        // Stop extending rather than publish a hole: every later point is
        // further into the same unreachable stretch, and a trajectory whose
        // interior is wrong is worse than a shorter one.
        if(!usable)
        {
            break;
        }

        lookahead_[lookahead_count_].time_from_start = offset;
        for(size_t i = 0; i < 12; i++)
        {
            lookahead_[lookahead_count_].positions[i] = future_joints[i];
        }
        lookahead_count_++;

        if(offset >= lookahead_horizon_)
        {
            break;
        }

        offset *= 2.0;
    }

    leg_controller_.restoreGaitState(saved);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{   
    
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z +  gait_config_.nominal_height;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{
    const rclcpp::Time now = clock_.now();

    if(publish_joint_control_)
    {
        // MEASURED sim-time gap since the last command actually went out. This
        // is the crux of the load-correlated freeze: it is not loop_period_.
        // When the machine is oversubscribed gz stalls on a render pass and
        // then bursts sim time, so the sim-time loop timer fires once per
        // BURST rather than once per nominal period. Commands then arrive
        // 30-80 ms of sim time apart while the trajectory told the gz
        // JointTrajectoryController to finish its ramp in 10 ms — the JTC hit
        // the target, ran out of trajectory, and held. A 10 ms move followed
        // by a 40 ms hold, cycling at the render rate, IS the freeze.
        double interval = loop_period_;
        const bool have_previous = !std::isnan(last_published_joints_[0]);
        if(have_previous)
        {
            interval = (now - last_publish_time_).seconds();
        }

        // Travel budget for the slew limiter ONLY — the ramp is sized from the
        // move itself further down, not from this. After a long gap the leg
        // legitimately has more ground to cover, so the allowance scales with
        // the gap that actually elapsed; capping it at max_command_interval_
        // bounds how much catch-up any single command may attempt.
        //
        // Flooring at loop_period_ keeps nominal behavior identical to before
        // and makes a zero or negative interval (sim-time rewind on reset)
        // harmless. The !(x > y) form also catches NaN.
        double effective = interval;
        if(!(effective > loop_period_))
            effective = loop_period_;
        if(effective > max_command_interval_)
            effective = max_command_interval_;

        if(have_previous && timing_report_period_ > 0.0)
        {
            interval_sum_ += interval;
            interval_count_++;
            if(interval > interval_worst_)
                interval_worst_ = interval;
            if(interval > 2.0 * loop_period_)
                interval_late_++;

            // Reseed rather than stall forever if sim time jumped backwards.
            if(now + rclcpp::Duration::from_seconds(timing_report_period_) < next_timing_report_)
                next_timing_report_ = now;

            if(now >= next_timing_report_ && interval_count_ > 0)
            {
                RCLCPP_INFO(this->get_logger(),
                    "joint cmd interval (nominal %.1f ms): mean %.1f ms, worst %.1f ms, "
                    "%.1f%% over 2x nominal [n=%lu]",
                    1e3 * loop_period_,
                    1e3 * interval_sum_ / static_cast<double>(interval_count_),
                    1e3 * interval_worst_,
                    100.0 * static_cast<double>(interval_late_) /
                        static_cast<double>(interval_count_),
                    interval_count_);

                interval_sum_ = 0.0;
                interval_worst_ = 0.0;
                interval_count_ = 0;
                interval_late_ = 0;
                next_timing_report_ =
                    now + rclcpp::Duration::from_seconds(timing_report_period_);
            }
        }

        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        // Zero stamp on purpose: it tells the gz JointTrajectoryController to
        // start the trajectory on arrival rather than at an absolute sim time,
        // so time_from_start below is measured from when it lands.
        joints_cmd_msg.header.stamp.sec = 0;
        joints_cmd_msg.header.stamp.nanosec = 0;

        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        // Slew limit. When a burst in /clock jumps the gait phase, the foot
        // target can leave leg reach for a cycle or two; IK aborts, the pose is
        // held, and phase keeps advancing. The pose commanded on the cycle
        // reach is regained is therefore discontinuous, and against the leg PID
        // (p=600) that lands as a slam rather than a placement — the foot hits
        // unevenly and the next stride topples the robot.
        //
        // Capping the change ramps back onto the trajectory instead of stepping
        // onto it. Deliberately clamped against the last PUBLISHED target, not
        // last_valid_joints_: consecutive clamped cycles must chain, otherwise a
        // multi-cycle catch-up re-introduces the jump on cycle two.
        const float max_delta =
            static_cast<float>(max_joint_velocity_ * effective);
        bool clamped = false;
        float largest_delta = 0.0f;
        for(size_t i = 0; i < 12; i++)
        {
            float commanded = target_joints[i];
            if(!std::isnan(last_published_joints_[i]))
            {
                const float delta = commanded - last_published_joints_[i];
                if(delta > max_delta)
                {
                    commanded = last_published_joints_[i] + max_delta;
                    clamped = true;
                }
                else if(delta < -max_delta)
                {
                    commanded = last_published_joints_[i] - max_delta;
                    clamped = true;
                }

                const float moved = std::fabs(commanded - last_published_joints_[i]);
                if(moved > largest_delta)
                    largest_delta = moved;
            }
            point.positions[i] = commanded;
            last_published_joints_[i] = commanded;
        }

        // Ramp is loop_period_ in the common case — the measured mean interval
        // really is ~10 ms, and sizing the ramp from the PREVIOUS interval
        // instead made the leg crawl for a cycle after every isolated spike.
        // It stretches only when the move genuinely needs longer, which keeps
        // commanded velocity at or below max_joint_velocity_ without ever
        // slowing a normal step: a 0.2 rad move at 20 rad/s needs exactly the
        // 10 ms it already has.
        double ramp = loop_period_;
        const double required = static_cast<double>(largest_delta) / max_joint_velocity_;
        if(required > ramp)
            ramp = required;
        point.time_from_start = rclcpp::Duration::from_seconds(ramp);

        joints_cmd_msg.points.push_back(point);

        // Lookahead points: the actual gait, evaluated forward. These are what
        // keep the leg moving when a command is late. Skipped entirely on a
        // clamped cycle — the slew limiter only engages on a discontinuity, and
        // extrapolating away from a target we just refused to jump to would
        // hand the JTC the very motion the clamp exists to prevent.
        double last_time = ramp;
        if(!clamped)
        {
            for(size_t k = 0; k < lookahead_count_; k++)
            {
                if(lookahead_[k].time_from_start <= last_time)
                    continue;

                trajectory_msgs::msg::JointTrajectoryPoint future;
                future.positions.assign(lookahead_[k].positions.begin(),
                                        lookahead_[k].positions.end());
                future.time_from_start =
                    rclcpp::Duration::from_seconds(lookahead_[k].time_from_start);
                joints_cmd_msg.points.push_back(future);
                last_time = lookahead_[k].time_from_start;
            }
        }

        // Terminal hold, past the lookahead horizon. Repeating the LAST point
        // rather than extrapolating is deliberate: beyond the horizon we have
        // stopped predicting, so a gap that long should degrade to holding a
        // real gait pose instead of drifting somewhere nobody planned. With
        // lookahead present this is only reached after a gap longer than the
        // horizon; without it, this is the old behaviour unchanged.
        trajectory_msgs::msg::JointTrajectoryPoint hold = joints_cmd_msg.points.back();
        hold.time_from_start =
            rclcpp::Duration::from_seconds(last_time + 3.0 * loop_period_);
        joints_cmd_msg.points.push_back(hold);
        joint_commands_publisher_->publish(joints_cmd_msg);

        last_publish_time_ = now;
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::msg::JointState joints_msg;

        joints_msg.header.stamp = now;

        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= target_joints[i];
        }

        joint_states_publisher_->publish(joints_msg);
    }
}

void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = clock_.now();
        contacts_msg.contacts.resize(4);
        
        std::string s2;
       for(size_t i = 0; i < 4; i++)
        {
            //This is only published when there's no feedback on the robot
            //that a leg is in contact with the ground
            //For such cases, we use the stance phase in the gait for foot contacts
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
            s2.append(std::to_string(contacts_msg.contacts[i]) + " ");
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}
