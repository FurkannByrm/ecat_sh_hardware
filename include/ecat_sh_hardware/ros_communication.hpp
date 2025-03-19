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
 #include "ecat_sh_hardware/controller.hpp"
 
 #include <rclcpp/rclcpp.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_msgs/msg/tf_message.hpp>
 #include "sensor_msgs/msg/battery_state.hpp"
 #include <nav_msgs/msg/odometry.hpp>
 #include <geometry_msgs/msg/twist_stamped.hpp>
 #include <sensor_msgs/msg/joint_state.hpp>
 #include "ipc_handlers/tcp_client_template.hpp"
 #include <amr_custom_interfaces/msg/hardware_info_array.hpp>
 #include "geometry_msgs/msg/transform_stamped.hpp"
 #include "tf2/LinearMath/Quaternion.h"
 #include "tf2_ros/transform_broadcaster.h"
 #include "ecat_sh_hardware/bms_uart.hpp"
 #include <nlohmann/json.hpp>
 
 
 #include <queue>
 #include <memory>
 #include <atomic>
 #include <future>
 #include <vector>
 #include <expected>
 #include <chrono>
 #include <string_view>
 #include <string>
 
 
 
 struct RosData
 {
   Odometry odometry;
   std::vector<CIA402_State> device_states;
   std::vector<JointInfo> joint_states;
 
 };
 enum class LedMode : uint16_t {
   OFF = 0x0, // 000
   GREEN = 0x1, // 100
   BLUE = 0x2 , // 010
   RED = 0x4, // 001
   BOUNCING_GREEN = 0x3, // 110
   BOUNCING_BLUE = 0x5, // 101
   BOUNCING_RED = 0x6, // 011
   CLUB_EFFECT = 0x7 // 111
 };
 
 enum class HardwareState : uint16_t {
   INITIAL_STATE = 0x7,
   OPERATION_ENABLED = 0x5,
   STAMPED_MOVE = 0x2,
   CHARGING_MODE = 0x3,
   CHARGING_COMPLETE = 0x1,
   ERROR = 0x4,
   WARNING = 0x4
 };
 
 class BatteryStatus : public rclcpp::Node{
 
 
     public:
     BatteryStatus();
 
 
     private:
     void BatteryStatusCallBack();
     rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     BMS_UART bms_; 
     
     // const int MAX_TEMP_THRESHOLD_;
     // std::string STATUS_;  
 
     // enum class BatteryChargeState{
     //     STATIONARY, 
     //     CHARGING,
     //     DECHARGING 
 
     // };
 };
 
 
 
 
 
 
 
 struct VelocityCommand
 {
   double linear = 0.0;
   double angular = 0.0;
 };
 
 struct IoRequest
 {
   public:
 
   enum class RequestType {
     READ,
     WRITE
   };
 
   using Request = std::tuple<int, RequestType, std::optional<int>>;
 
   std::chrono::time_point<std::chrono::system_clock> timestamp;
   std::vector<Request> requests; 
 
   static std::string toJsonStr(const IoRequest& data) {
     nlohmann::json j;
     j["timestamp"] = std::chrono::system_clock::to_time_t(data.timestamp);
     for (const auto& [key, type, value] : data.requests) {
       nlohmann::json req;
       req["key"] = key;
       req["type"] = type == RequestType::READ ? "READ" : "WRITE";
       if (value) {
         req["value"] = *value;
       }
       j["requests"].push_back(req);
     }
     return j.dump();
   }
 
   static std::optional<IoRequest> fromStr(const std::string& str) {
     nlohmann::json j = nlohmann::json::parse(str);
     if(j.empty())
     {
       return std::nullopt;
     }
 
     IoRequest data;
 
     data.timestamp = std::chrono::system_clock::from_time_t(j["timestamp"].get<std::time_t>());
     for (const auto& req : j["requests"]) {
       RequestType type = req["type"] == "READ" ? RequestType::READ : RequestType::WRITE;
       std::optional<int> value;
       if (req.find("value") != req.end()) {
         value = req["value"].get<int>();
       }
       data.requests.push_back({req["key"].get<int>(), type, value});
     }
     return data;
   }
 
 };
 
 void setupOdometryMsg(nav_msgs::msg::Odometry& odom_msg);
 
 void toRosOdom(const Odometry& odom, nav_msgs::msg::Odometry& ros_odom);
 
 void ros_communication(std::atomic<bool>& shutdown_requested, std::mutex& ros_sync_mutex, std::shared_ptr<VelocityCommand>& command_ptr, std::shared_ptr<RosData>& data);
 
 #endif // ROS_COMMUNICATION_HPP_