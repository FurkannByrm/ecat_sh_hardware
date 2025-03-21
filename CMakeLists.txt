cmake_minimum_required(VERSION 3.3.0)
project(ecat_sh_hardware VERSION 0.1 LANGUAGES C CXX)

set(CXX_STANDARD 23)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/upxtreme/ethercat/include/ /usr/local/ /opt/ros/humble/include/)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra  -Werror=return-type)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++23)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED COMPONENTS LinearMath)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(amr_custom_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ipc_handlers REQUIRED)
find_package(timespec_chrono_conversions REQUIRED)

add_executable(
	diff_drive_hardware
	src/ecat_hardware.cpp
  src/io_tcp_server.cpp
  src/utils.cpp
)
target_include_directories(diff_drive_hardware PUBLIC include/ /home/upxtreme/ethercat/include/ ${ipc_handlers_INCLUDE_DIRS} ${timespec_chrono_conversions_INCLUDE_DIRS}/timespec_chrono_conversions)
target_link_libraries(diff_drive_hardware /usr/local/etherlab/lib/libethercat.so rt Threads::Threads ${ipc_handlers_LIBRARIES})

add_executable(
  diff_drive_controller
  src/controller.cpp
  src/kinematics.cpp
  src/ros_communication.cpp
  src/bms_uart.cpp
)
target_link_libraries(diff_drive_controller rt Threads::Threads ${ipc_handlers_LIBRARIES} ${rclcpp_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${rosidl_typesupport_cpp_LIBRARIES} ${rosidl_default_runtime_LIBRARIES} ${tf2_LIBRARIES} ${geometry_msgs} ${sensor_msgs_LIBRARIES} ${amr_custom_interfaces_LIBRARIES} ${nav_msgs_LIBRARIES} ${tf2_msgs} ${tf2_ros} tf2_ros::tf2_ros)
target_include_directories(
  diff_drive_controller 
  PUBLIC 
  include/ 
  ${Boost_INCLUDE_DIRS}
  ${ipc_handlers_INCLUDE_DIRS} 
  ${timespec_chrono_conversions_INCLUDE_DIRS}/timespec_chrono_conversions 
  ${rclcpp_INCLUDE_DIRS} 
  ${rosidl_typesupport_cpp_INCLUDE_DIRS} 
  ${tf2_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS} 
  ${nav_msgs_INCLUDE_DIRS} 
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${amr_custom_interfaces_INCLUDE_DIRS}
)

install(
	TARGETS
	diff_drive_hardware diff_drive_controller
	DESTINATION
	${CMAKE_CURRENT_BINARY_DIR}/build
)


