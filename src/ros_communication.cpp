
#include "ecat_sh_hardware/ros_communication.hpp"

void setupOdometryMsg(nav_msgs::msg::Odometry& odom_msg)
{
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.w = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  constexpr std::array<double, 6> positionCovArr = { 0.001, 0.001, 0.001, 0.001, 0.001, 0.01 };
  constexpr std::array<double, 6> velocityCovArr = { 0.001, 0.001, 0.001, 0.001, 0.001, 0.01 };

  for (std::size_t i = 0; i < 6; i++)
  {
    const std::size_t diag_index = 6 * i + i;
    odom_msg.pose.covariance.at(diag_index) = positionCovArr.at(i);
    odom_msg.twist.covariance.at(diag_index) = velocityCovArr.at(i);
  }
}

void toRosOdom(const Odometry& odom, nav_msgs::msg::Odometry& ros_odom)
{
  ros_odom.pose.pose.position.x = odom.x;
  ros_odom.pose.pose.position.y = odom.y;
  auto quatTf2 = tf2::Quaternion();
  quatTf2.setRPY(0.0, 0.0, odom.heading);
  ros_odom.pose.pose.orientation.x = quatTf2.getX();
  ros_odom.pose.pose.orientation.y = quatTf2.getY();
  ros_odom.pose.pose.orientation.z = quatTf2.getZ();
  ros_odom.pose.pose.orientation.w = quatTf2.getW();
  ros_odom.twist.twist.linear.x = odom.linearVel;
  ros_odom.twist.twist.angular.z = odom.angularVel;
}

void ros_communication(std::atomic<bool>& shutdown_requested, std::mutex& ros_sync_mutex,
                       std::shared_ptr<VelocityCommand>& command_ptr, std::shared_ptr<RosData>& data)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Node> controllerNode = std::make_shared<rclcpp::Node>("diff_drive_controller_node");

  std::shared_ptr<VelocityCommand> velCommandPtr = command_ptr;
  std::shared_ptr<RosData> rosDataPtr = data;
  std::queue<geometry_msgs::msg::TwistStamped> velCmdQueue;
  
  RosData rosData;
  nav_msgs::msg::Odometry odomMsg;
  setupOdometryMsg(odomMsg);

  amr_custom_interfaces::msg::HardwareInfoArray hardwareInfoMsg;
  hardwareInfoMsg.motor_driver_info.resize(2);

  sensor_msgs::msg::JointState jointStateMsg;
  jointStateMsg.name.resize(2);
  jointStateMsg.name[0] = "right_wheel_joint";
  jointStateMsg.name[1] = "left_wheel_joint";
  jointStateMsg.position.resize(2);
  jointStateMsg.position[0] = 0.0;
  jointStateMsg.position[1] = 0.0;
  jointStateMsg.velocity.resize(2);
  jointStateMsg.velocity[0] = 0.0;
  jointStateMsg.velocity[1] = 0.0;
  jointStateMsg.header.frame_id = "base_link";
  

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> velCommandSub =
      controllerNode->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/cmd_vel", rclcpp::SystemDefaultsQoS(),
          [&velCmdQueue](std::shared_ptr<geometry_msgs::msg::TwistStamped> vel_cmd) {
            // Push to command queue
            velCmdQueue.push(*vel_cmd);
          });

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub =
  controllerNode->create_publisher<nav_msgs::msg::Odometry>("/odom",
                                                                rclcpp::SystemDefaultsQoS());

  std::shared_ptr<rclcpp::Publisher<amr_custom_interfaces::msg::HardwareInfoArray>> hardwareInfoPub =
      controllerNode->create_publisher<amr_custom_interfaces::msg::HardwareInfoArray>(
          "/hardware_info", rclcpp::SystemDefaultsQoS());

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> jointStatePub =
      controllerNode->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                                     rclcpp::SystemDefaultsQoS());

  auto cleanupRosMembers = [&]() -> void {
    velCommandSub.reset();
    odomPub.reset();
    hardwareInfoPub.reset();
    controllerNode.reset();
  };

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(controllerNode);
  rclcpp::Rate rate(250.0);

  rclcpp::Time previousUpdateTime = controllerNode->get_clock()->now();
  const rclcpp::Duration velocityCommandTimeout = rclcpp::Duration::from_seconds(0.1);

  while (!shutdown_requested.load() && rclcpp::ok())
  {
    exec.spin_some();
    rclcpp::Time currentTime = controllerNode->get_clock()->now();
    ros_sync_mutex.lock();
    const auto timeElapsedSinceLatestUpdate = currentTime - previousUpdateTime;
    if (!velCmdQueue.empty())
    {

      if ((timeElapsedSinceLatestUpdate <= velocityCommandTimeout))
      {
        auto currCommand = velCmdQueue.front().twist;
        *command_ptr = { currCommand.linear.x, currCommand.angular.z };  
      }
      auto currCommand = velCmdQueue.front().twist;
        
      *command_ptr = { currCommand.linear.x, currCommand.angular.z };  
      velCmdQueue.pop();
    }

    rosData = *data;
    ros_sync_mutex.unlock();

    hardwareInfoMsg.motor_driver_info.at(0).status = to_integral(rosData.device_states.at(0));
    hardwareInfoMsg.motor_driver_info.at(1).status = to_integral(rosData.device_states.at(1));
    hardwareInfoMsg.motor_driver_info.at(0).current_position = rosData.joint_states.at(0).position;
    hardwareInfoMsg.motor_driver_info.at(1).current_position = rosData.joint_states.at(1).position;
    hardwareInfoMsg.motor_driver_info.at(0).current_velocity = rosData.joint_states.at(0).velocity;
    hardwareInfoMsg.motor_driver_info.at(1).current_velocity = rosData.joint_states.at(1).velocity;

    jointStateMsg.position[0] = rosData.joint_states.at(0).position;
    jointStateMsg.position[1] = rosData.joint_states.at(1).position;
    jointStateMsg.velocity[0] = rosData.joint_states.at(0).velocity;
    jointStateMsg.velocity[1] = rosData.joint_states.at(1).velocity;
    jointStateMsg.header.stamp = controllerNode->get_clock()->now();
    
    toRosOdom(rosData.odometry, odomMsg);

    odomMsg.header.stamp = currentTime;
    odomPub->publish(odomMsg);
    jointStatePub->publish(jointStateMsg);
    hardwareInfoPub->publish(hardwareInfoMsg);  

    previousUpdateTime = currentTime;
    rate.sleep();
  }


  cleanupRosMembers();

  rclcpp::shutdown();
  
}