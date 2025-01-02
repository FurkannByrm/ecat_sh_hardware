/**
 * @file ros_communication.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ROS_COMMUNICATION_HPP_
#define ROS_COMMUNICATION_HPP_

#include "ecat_sh_hardware/kinematics.hpp"
#include "ecat_sh_hardware/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <amr_custom_interfaces/msg/hardware_info_array.hpp>

#include <queue>
#include <memory>
#include <atomic>
#include <future>
#include <vector>

struct RosData
{
  Odometry odometry;
  std::vector<CIA402_State> device_states;
  std::vector<JointInfo> joint_states;

};

struct VelocityCommand
{
  double linear = 0.0;
  double angular = 0.0;
};

void setupOdometryMsg(nav_msgs::msg::Odometry& odom_msg);

void toRosOdom(const Odometry& odom, nav_msgs::msg::Odometry& ros_odom);

void ros_communication(std::atomic<bool>& shutdown_requested, std::mutex& ros_sync_mutex, std::shared_ptr<VelocityCommand>& command_ptr, std::shared_ptr<RosData>& data);

#endif // ROS_COMMUNICATION_HPP_