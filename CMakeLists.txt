cmake_minimum_required(VERSION 3.3.0)
project(ecat_sh_hardware VERSION 0.1 LANGUAGES C CXX)

set(CXX_STANDARD 17)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra  -Werror=return-type)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++17)

find_package(Threads REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED COMPONENTS LinearMath)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(
	diff_drive_hardware
	src/ecat_hardware.cpp
  src/shared_memory_handler.cpp
  src/utils.cpp
)
target_include_directories(diff_drive_hardware PUBLIC include/)
target_link_libraries(diff_drive_hardware ethercat rt Threads::Threads)

add_executable(
  diff_drive_controller
  src/controller.cpp
  src/shared_memory_handler.cpp
  src/kinematics.cpp
  src/ros_communication.cpp
)
target_link_libraries(diff_drive_controller rt Threads::Threads ${rclcpp_LIBRARIES} ${rcl_interfaces_LIBRARIES} ${rosidl_typesupport_cpp_LIBRARIES} ${rosidl_default_runtime_LIBRARIES} ${tf2_LIBRARIES} ${geometry_msgs} ${nav_msgs_LIBRARIES} ${tf2_msgs})
target_include_directories(diff_drive_controller PUBLIC include/ ${rclcpp_INCLUDE_DIRS} ${rosidl_typesupport_cpp_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${tf2_msgs_INCLUDE_DIRS})

install(
	TARGETS
	diff_drive_hardware diff_drive_controller
	DESTINATION
	${CMAKE_CURRENT_BINARY_DIR}/build
)

