
#include "ecat_sh_hardware/ros_communication.hpp"

void setupOdometryMsg(nav_msgs::msg::Odometry& odom_msg)
{
  
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

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> velCommandSub =
      controllerNode->create_subscription<geome y_msgs::msg::TwistStamped>(
          "/diff_drive_controller/velocity_command", rclcpp::SystemDefaultsQoS(),
          [&velCmdQueue](std::shared_ptr<geometry_msgs::msg::TwistStamped> vel_cmd) {
            // Push to command queue
            velCmdQueue.push(*vel_cmd);
          });
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub =
      controllerNode->create_publisher<nav_msgs::msg::Odometry>("/diff_drive_controller/odometry",
                                                                rclcpp::SystemDefaultsQoS());

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controllerNode);
  rclcpp::Rate rate(250.0);

  while (!shutdown_requested.load() && rclcpp::ok())
  {
    exec.spin_once();
    ros_sync_mutex.lock();

    if (!velCmdQueue.empty())
    {
      auto currCommand = velCmdQueue.front().twist;
      *command_ptr = { currCommand.linear.x, currCommand.angular.z };
      velCmdQueue.pop();
    }
    rosData = *data;
    ros_sync_mutex.unlock();
    toRosOdom(rosData.odometry, odomMsg);
    odomPub->publish(odomMsg);
  }

  rate.sleep();
}