cmake_minimum_required(VERSION 3.1.0)
project(ecat_sh_hardware VERSION 0.1 LANGUAGES C CXX)

set(CXX_STANDARD 20)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type)
endif()



add_executable(
	diff_drive_hardware
	src/ecat_hardware.cpp
  src/shared_memory_handler.cpp
  src/utils.cpp
)
target_include_directories(diff_drive_hardware PUBLIC include/)
target_link_libraries(diff_drive_hardware ethercat)
install(
	TARGETS
	diff_drive_hardware
	DESTINATION
	${CMAKE_CURRENT_SOURCE_DIR}/build
)
