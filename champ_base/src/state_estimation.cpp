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

#include <state_estimation.h>

champ::Odometry::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

StateEstimation::StateEstimation():
    Node("state_estimation_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    odometry_(base_, rosTimeToChampTime(clock_.now()))
{
    last_vel_time_ = clock_.now();
    last_sync_time_ = clock_.now();
    base_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sync_callback_group_       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    pose_timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sync_sub_options;
    sync_sub_options.callback_group = sync_callback_group_;

    joint_states_subscriber_.subscribe(
        reinterpret_cast<rclcpp::Node*>(this), "joint_states",
        rmw_qos_profile_default, sync_sub_options);
    foot_contacts_subscriber_.subscribe(
        reinterpret_cast<rclcpp::Node*>(this), "foot_contacts",
        rmw_qos_profile_default, sync_sub_options);

    this->sync = std::make_unique<Sync>(
        SyncPolicy(10), 
        this->joint_states_subscriber_, 
        this->foot_contacts_subscriber_
    );
    
    // Register callback functions
    this->sync->registerCallback(
        std::bind(
            &StateEstimation::synchronized_callback_, 
            this,
            std::placeholders::_1, std::placeholders::_2
        )
    );

    footprint_to_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/raw", 1);
    base_to_footprint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("base_to_footprint_pose", 1);
    foot_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("foot", 1);
    
    std::string urdf = "";

    orientation_from_imu_ = false;

    this->get_parameter("links_map.base",    base_name_);
    this->get_parameter("gait.odom_scaler",  gait_config_.odom_scaler);
    this->get_parameter("orientation_from_imu",  orientation_from_imu_);

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("urdf",                        urdf);

    if (orientation_from_imu_)
    {
      rclcpp::SubscriptionOptions imu_sub_options;
      imu_sub_options.callback_group = imu_callback_group_;

      imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 1, std::bind(&StateEstimation::imu_callback_, this,  std::placeholders::_1),
        imu_sub_options);
    }
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());

    node_namespace_ = this->get_namespace();
    if(node_namespace_.length() > 1)
    {
        node_namespace_.replace(0, 1, "");
        node_namespace_.push_back('/');
    }
    else
    {
        node_namespace_ = "";
    }

    odom_frame_ = node_namespace_ + "odom";
    base_footprint_frame_ = node_namespace_ + "base_footprint";
    base_link_frame_ = node_namespace_ + base_name_;

    std::chrono::milliseconds period(static_cast<int>(1000/50));

    odom_data_timer_ = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::milliseconds>(period),
         std::bind(&StateEstimation::publishFootprintToOdom_, this),
         odom_timer_callback_group_);

    base_pose_timer_ = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::milliseconds>(period),
         std::bind(&StateEstimation::publishBaseToFootprint_, this),
         pose_timer_callback_group_);
}

void StateEstimation::synchronized_callback_(const std::shared_ptr<sensor_msgs::msg::JointState const>& joints_msg, 
                                const std::shared_ptr<champ_msgs::msg::ContactsStamped const>& contacts_msg)
{
    last_sync_time_ = clock_.now();
    
    float current_joint_positions[12];

    for(size_t i = 0; i < joints_msg->name.size(); i++)
    {
        auto itr = std::find(joint_names_.begin(), joint_names_.end(), joints_msg->name[i]);
        if (itr == joint_names_.end())
            continue;  // skip joints CHAMP doesn't track (arm, gripper, etc.)
        int index = std::distance(joint_names_.begin(), itr);
        current_joint_positions[index] = joints_msg->position[i];
    }

    std::lock_guard<std::mutex> lock(base_mutex_);

    base_.updateJointPositions(current_joint_positions);

    for(size_t i = 0; i < 4; i++)
    {
        base_.legs[i]->in_contact(contacts_msg->contacts[i]);
    }
}

void StateEstimation::imu_callback_(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  last_imu_ = msg;
}

void StateEstimation::publishFootprintToOdom_()
{
    {
        std::lock_guard<std::mutex> lock(base_mutex_);
        odometry_.getVelocities(current_velocities_, rosTimeToChampTime(clock_.now()));
    }

    rclcpp::Time current_time = clock_.now();

    double vel_dt = (current_time - last_vel_time_).nanoseconds()/1e-9;
    last_vel_time_ = current_time;
    //rotate in the z axis
    //https://en.wikipedia.org/wiki/Rotation_matrix
    double delta_heading = current_velocities_.angular.z * vel_dt; 
    double delta_x = (current_velocities_.linear.x * cos(heading_) - current_velocities_.linear.y * sin(heading_)) * vel_dt; //m
    double delta_y = (current_velocities_.linear.x * sin(heading_) + current_velocities_.linear.y * cos(heading_)) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading_ in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, heading_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_footprint_frame_;

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading_ in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[35] = 0.017;

    odom.twist.twist.linear.x = current_velocities_.linear.x;
    odom.twist.twist.linear.y = current_velocities_.linear.y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = current_velocities_.angular.z;

    odom.twist.covariance[0] = 0.3;
    odom.twist.covariance[7] = 0.3;
    odom.twist.covariance[35] = 0.017;
    
    footprint_to_odom_publisher_->publish(odom);
}

visualization_msgs::msg::Marker StateEstimation::createMarker_(geometry::Transformation foot_pos, int id, std::string frame_id)
{
    visualization_msgs::msg::Marker foot_marker;

    foot_marker.header.frame_id = frame_id;

    foot_marker.type = visualization_msgs::msg::Marker::SPHERE;
    foot_marker.action = visualization_msgs::msg::Marker::ADD;
    foot_marker.id = id;

    foot_marker.pose.position.x = foot_pos.X();
    foot_marker.pose.position.y = foot_pos.Y();
    foot_marker.pose.position.z = foot_pos.Z();
    
    foot_marker.pose.orientation.x = 0.0;
    foot_marker.pose.orientation.y = 0.0;
    foot_marker.pose.orientation.z = 0.0;
    foot_marker.pose.orientation.w = 1.0;

    foot_marker.scale.x = 0.025;
    foot_marker.scale.y = 0.025;
    foot_marker.scale.z = 0.025;

    foot_marker.color.r = 0.780;
    foot_marker.color.g = 0.082;
    foot_marker.color.b = 0.521;
    foot_marker.color.a = 0.5;

    return foot_marker;
}

void StateEstimation::publishBaseToFootprint_()
{
    {
        std::lock_guard<std::mutex> lock(base_mutex_);
        base_.getFootPositions(current_foot_positions_);
    }

    // Foot position markers are kept purely for RViz debug visualization;
    // this no longer depends on contact state since orientation/height
    // are both fixed below (see notes on kManualFootprintHeight and
    // kIdentityOrientation).
    if (foot_publisher_->get_subscription_count())
    {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < 4; i++)
        {
            marker_array.markers.push_back(createMarker_(current_foot_positions_[i], i, base_link_frame_));
        }
        foot_publisher_->publish(marker_array);
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = base_footprint_frame_;
    pose_msg.header.stamp = clock_.now();

    pose_msg.pose.covariance[0] = 0.001;
    pose_msg.pose.covariance[7] = 0.001;
    pose_msg.pose.covariance[14] = 0.001;
    pose_msg.pose.covariance[21] = 0.0001;
    pose_msg.pose.covariance[28] = 0.0001;
    pose_msg.pose.covariance[35] = 0.017;

    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;

    constexpr float kManualFootprintHeight = 0.4680730700492859f;
    pose_msg.pose.pose.position.z = kManualFootprintHeight;

    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    base_to_footprint_publisher_->publish(pose_msg);
}