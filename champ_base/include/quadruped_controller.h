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

#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <champ_msgs/msg/joints.hpp>
#include <champ_msgs/msg/pose.hpp>
#include <champ_msgs/msg/point_array.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>

#include <champ/body_controller/body_controller.h>
#include <champ/utils/urdf_loader.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class QuadrupedController: public rclcpp::Node
{
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscription_;
    

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::Clock clock_;
    
    champ::Velocities req_vel_;
    champ::Pose req_pose_;

    champ::GaitConfig gait_config_;

    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;

    std::vector<std::string> joint_names_;

    bool publish_foot_contacts_;
    bool publish_joint_states_;
    bool publish_joint_control_;
    bool in_gazebo_;

    double loop_period_;
    std::array<float, 12> last_valid_joints_;

    // Slew limiting on the published joint targets. last_published_joints_ is
    // what actually went out on the wire last cycle (post-clamp), which is what
    // the rate limit must be measured against — last_valid_joints_ is raw IK
    // output and can jump.
    double max_joint_velocity_;
    std::array<float, 12> last_published_joints_;

    // Sim time at which the last joint command actually went out. The gap
    // between consecutive publishes is NOT loop_period_ under load — /clock
    // arrives in bursts and the sim-time loop timer fires once per burst — so
    // it is measured rather than assumed.
    rclcpp::Time last_publish_time_;

    // Upper bound in seconds on how much of a measured gap is honoured when
    // sizing the trajectory ramp and the slew allowance. Caps how far a single
    // command tries to catch up after a long stall.
    double max_command_interval_;

    // Command-interval instrumentation. Reported every timing_report_period_
    // seconds of sim time; set that parameter to 0 to disable.
    double timing_report_period_;
    double interval_sum_;
    double interval_worst_;
    unsigned long interval_count_;
    unsigned long interval_late_;
    rclcpp::Time next_timing_report_;

    // Trajectory lookahead. Gait phase is a pure function of time, so the foot
    // targets 10-150 ms from now are already known. Publishing them as extra
    // trajectory points means a command that arrives late finds a LIVE, moving
    // trajectory instead of an exhausted one — the leg keeps walking through
    // the gap rather than holding at a reached target. Under normal 10 ms
    // cycles the next command replaces the trajectory before any of these
    // points is reached, so nominal behaviour is unchanged.
    static constexpr size_t kMaxLookaheadPoints = 8;

    struct LookaheadSample
    {
        double time_from_start;
        std::array<float, 12> positions;
    };

    double lookahead_horizon_;
    int lookahead_points_;
    std::array<LookaheadSample, kMaxLookaheadPoints> lookahead_;
    size_t lookahead_count_;

    void controlLoop_();
    void buildLookahead_(const rclcpp::Time &now, const float current_joints[12]);
    
    void publishJoints_(float target_joints[12]);
    void publishFootContacts_(bool foot_contacts[4]);

    void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg);

    public:
        QuadrupedController();
};

#endif