cmake_minimum_required(VERSION 3.1.0)
project(ecat_sh_hardware VERSION 0.1 LANGUAGES C CXX)

set(CXX_STANDARD 17)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=unused-but-set-variable -Werror=return-type)
endif()

add_compile_options(-std=c++17)

find_package(Threads REQUIRED)

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
)
target_include_directories(diff_drive_controller PUBLIC include/)
target_link_libraries(diff_drive_controller rt Threads::Threads)


install(
	TARGETS
	diff_drive_hardware diff_drive_controller
	DESTINATION
	${CMAKE_CURRENT_SOURCE_DIR}/build
)
