
#include "ecat_sh_hardware/ros_communication.hpp"

std::shared_ptr<float> batteryCurrent = std::make_shared<float>(0.0);  
std::shared_ptr<float> batteryVoltage = std::make_shared<float>(0.0); 

BatteryStatus::BatteryStatus(): Node{"bms_status"},bms_{"/dev/ttyUSB0"}
{
    if (!bms_.Init())
    {
        RCLCPP_ERROR(this->get_logger(), "BMS initialization failed!");
        rclcpp::shutdown();
        return;
    }
    
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("bms_status",1000);
    timer_     = this->create_wall_timer(std::chrono::seconds(1),std::bind(&BatteryStatus::BatteryStatusCallBack,this));
}

void BatteryStatus::BatteryStatusCallBack()
{

    bms_.update();
    
    sensor_msgs::msg::BatteryState msg;
    msg.voltage                 = bms_.get.packVoltage;
    msg.current                 = bms_.get.packCurrent;

    *batteryCurrent = msg.current;
    *batteryVoltage = msg.voltage;
    msg.charge                  = bms_.get.resCapacitymAh / 1000.0;
    msg.percentage              = bms_.get.packSOC / 100.0;
    msg.temperature             = bms_.get.tempAverage;
    msg.capacity                = 46;
    msg.present                 = true;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.power_supply_status     = bms_.get.batteryStatus < 5 ? bms_.get.batteryStatus : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN; 
    publisher_->publish(msg);

}


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
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
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
  jointStateMsg.effort.resize(2);
  jointStateMsg.name[0] = "joint_2";
  jointStateMsg.name[1] = "joint_1";
  jointStateMsg.position.resize(2);
  jointStateMsg.position[0] = 0.0;
  jointStateMsg.position[1] = 0.0;
  jointStateMsg.velocity.resize(2);
  jointStateMsg.velocity[0] = 0.0;
  jointStateMsg.velocity[1] = 0.0;
  jointStateMsg.header.frame_id = "base_link";

  auto vel_cmd_steamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> velCommandSub =
      controllerNode->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/cmd_vel", rclcpp::SystemDefaultsQoS(),
          [&velCmdQueue](std::shared_ptr<geometry_msgs::msg::TwistStamped> vel_cmd) {
            velCmdQueue.push(*vel_cmd);
          });


  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub =
      controllerNode->create_publisher<nav_msgs::msg::Odometry>("/odom",
                                                                10);
                                                                

    auto stamped_twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> unstampedTwistSub =
    controllerNode->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
        [stamped_twist, &velCmdQueue, controllerNode](std::shared_ptr<geometry_msgs::msg::Twist> unstamped_twist) {
          stamped_twist->header.stamp = controllerNode->get_clock()->now();
          stamped_twist->twist = *unstamped_twist;
          velCmdQueue.push(*stamped_twist);
        });

        auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(controllerNode);

        auto odom_sub_ = controllerNode->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SystemDefaultsQoS(),
            [tf_broadcaster, controllerNode](const nav_msgs::msg::Odometry::SharedPtr msg) {
              geometry_msgs::msg::TransformStamped transformStamped;
      
              // transformStamped.header.stamp = controllerNode->get_clock()->now();
              transformStamped.header.frame_id = "odom";
              transformStamped.child_frame_id = "base_link";
      
              transformStamped.transform.translation.x = msg->pose.pose.position.x;
              transformStamped.transform.translation.y = msg->pose.pose.position.y;
              transformStamped.transform.translation.z = msg->pose.pose.position.z;
              transformStamped.transform.rotation = msg->pose.pose.orientation;
              tf_broadcaster->sendTransform(transformStamped);
            });

  auto bmsNode = std::make_shared<BatteryStatus>();
  std::shared_ptr<rclcpp::Publisher<amr_custom_interfaces::msg::HardwareInfoArray>> hardwareInfoPub =
      controllerNode->create_publisher<amr_custom_interfaces::msg::HardwareInfoArray>(
          "/hardware_info", rclcpp::SystemDefaultsQoS());

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> jointStatePub =
      controllerNode->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                                      rclcpp::SensorDataQoS());
  

  HardwareState generalState = HardwareState::INITIAL_STATE;
  HardwareState previousState = generalState;
  std::shared_ptr<rclcpp::Node> ledControllerNode = std::make_shared<rclcpp::Node>("led_controller");
  std::shared_ptr<rclcpp::Subscription<amr_custom_interfaces::msg::HardwareInfoArray>> hardwareInfoSub =
    ledControllerNode->create_subscription<amr_custom_interfaces::msg::HardwareInfoArray>(
        "/hardware_info", 10,
        [stamped_twist, vel_cmd_steamped, &velCmdQueue, &generalState,bmsNode,&batteryCurrent, &batteryVoltage](const amr_custom_interfaces::msg::HardwareInfoArray::SharedPtr msg) {
          for (const auto& driverInfo : msg->motor_driver_info) {
            HardwareState newState = HardwareState::INITIAL_STATE;
         
            if (*batteryCurrent < 0)
            {
            
            if (driverInfo.status == amr_custom_interfaces::msg::MotorDriverInfo::OPERATION_ENABLED) {
              newState = HardwareState::OPERATION_ENABLED;
              if (stamped_twist->twist.linear.x  || 
                  stamped_twist->twist.angular.z ||
                  vel_cmd_steamped->twist.linear.x ||
                  vel_cmd_steamped->twist.angular.z
                  ){
                newState = HardwareState::STAMPED_MOVE;
              }
            } else if (driverInfo.status == amr_custom_interfaces::msg::MotorDriverInfo::QUICK_STOP_ACTIVE) {
              newState = HardwareState::ERROR;
            }           
            
            }
            

          else{
            newState = HardwareState::CHARGING_MODE;
            if (*batteryVoltage > 28.5)
            {
            newState = HardwareState::CHARGING_COMPLETE;
            }
            
          }
        
            if (newState != generalState) {
              generalState = newState;
            }
          }
        });
        




  std::expected<tcp_client::TcpClient, tcp_client::ErrorCode> tcpClientExp = tcp_client::TcpClient::create("192.168.2.159", "3255");

  // if(!tcpClientExp.has_value())
  // {
  //   std::cout << tcp_client::ErrorCodeDescriptions.at(tcpClientExp.error()) << std::endl;
  //   return 2;
  // }
  auto tcpClient = std::move(tcpClientExp.value());
  uint16_t ledMode = to_integral(generalState);
  IoRequest ledModeReq;
  ledModeReq.requests.resize(6);

  auto cleanupRosMembers = [&]() -> void {
    velCommandSub.reset();
    unstampedTwistSub.reset();
    odomPub.reset();
    hardwareInfoPub.reset();
    controllerNode.reset();    
  };

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(controllerNode);
  exec.add_node(ledControllerNode);
  exec.add_node(bmsNode);
  rclcpp::Rate rate(10.0);

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
      auto timestamp = velCmdQueue.front().header.stamp;

      if (((currentTime - timestamp) <= velocityCommandTimeout))
      {
        auto currCommand = velCmdQueue.front().twist;
        *command_ptr = {currCommand.linear.x, currCommand.angular.z};
      }

      
      else
      {
        *command_ptr = {0.0, 0.0};
      }
      velCmdQueue.pop();
    }
    /*     else
    {
      *command_ptr = { 0.0, 0.0 };
    } */
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
    jointStateMsg.effort[0] = 0;
    jointStateMsg.effort[1] = 0;



     if(generalState != previousState)
    {
      ledMode = to_integral(generalState);

      previousState = generalState;
    }
    
    
    
    ledModeReq.timestamp = std::chrono::system_clock::now(); 
    ledModeReq.requests[0] = std::make_tuple(0, IoRequest::RequestType::WRITE, (ledMode & 1));
    ledModeReq.requests[1] = std::make_tuple(1, IoRequest::RequestType::WRITE, ((ledMode & 2) >> 1));
    ledModeReq.requests[2] = std::make_tuple(2, IoRequest::RequestType::WRITE, (ledMode & 4) >> 2);
    
    ledModeReq.requests[3] = std::make_tuple(8, IoRequest::RequestType::WRITE, (ledMode & 1));
    ledModeReq.requests[4] = std::make_tuple(9, IoRequest::RequestType::WRITE, ((ledMode & 2) >> 1));
    ledModeReq.requests[5] = std::make_tuple(10, IoRequest::RequestType::WRITE, (ledMode & 4) >> 2);
    
  
    const std::string ledModeJsonReq = IoRequest::toJsonStr(ledModeReq);
    std::cout << ledModeJsonReq << std::endl;
    tcp_client::ErrorCode tcpSendRes = tcpClient.sendData(ledModeJsonReq.c_str(), ledModeJsonReq.length());
    std::cout << tcp_client::ErrorCodeDescriptions.at(tcpSendRes) << std::endl;


    // RLCPP_INFO(this->get_logger(), "hardware status",hardwareInfoMsg.motor_driver_info.at(0).status);
    toRosOdom(rosData.odometry, odomMsg);

    odomMsg.header.stamp = currentTime;
    odomPub->publish(odomMsg);
    jointStatePub->publish(jointStateMsg);
    hardwareInfoPub->publish(hardwareInfoMsg);

    previousUpdateTime = currentTime;
    rate.sleep();
  }

  cleanupRosMembers();

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(50ms);

  rclcpp::shutdown();
}
