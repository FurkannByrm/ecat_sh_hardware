/**
 * @file controller.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "ecat_sh_hardware/ros_communication.hpp"
#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecat_sh_hardware/utils.hpp"

#include <thread>
#include <mutex>
#include <algorithm>  
#include <future>
#include <map>
#include <functional>
#include <iostream>
#include <limits>

#include <string.h>
#include <errno.h>

#include "ecat_sh_hardware/kinematics.hpp"
#include "ipc_handlers/shm_handler.hpp"
#include "ipc_handlers/common.hpp"

struct VelocityLimiter
{
  double max_vel;
  double min_vel;
  double max_acc;
  double min_acc;
  double max_jerk;
  double min_jerk;
  double prev_command;

  VelocityLimiter();

  void limit(double& command, double dt, double current_value = std::numeric_limits<double>::quiet_NaN());
};



const std::map<CIA402_State, std::string> DEVICE_STATE_STR = {
  { CIA402_State::START, "Start" },
  { CIA402_State::NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On" },
  { CIA402_State::SWITCH_ON_DISABLED, "Switch on Disabled" },
  { CIA402_State::READY_TO_SWITCH_ON, "Ready to Switch On" },
  { CIA402_State::SWITCH_ON, "Switch On" },
  { CIA402_State::OPERATION_ENABLED, "Operation Enabled" },
  { CIA402_State::QUICK_STOP_ACTIVE, "Quick Stop Active" },
  { CIA402_State::FAULT_REACTION_ACTIVE, "Fault Reaction Active" },
  { CIA402_State::FAULT, "Fault" },
  { CIA402_State::UNKNOWN_STATE, "Unknown State" }
};

CIA402_State deriveState(uint16_t status_word);

uint16_t transitionToState(CIA402_State state, uint16_t control_word);

struct DriverStateHandler
{
  uint16_t previousControlWord;
  uint16_t previousStatusWord;
  CIA402_State currentState;
  CIA402_State previousState;

  bool isOperational = false;

  DriverStateHandler()
  {
    previousControlWord = 0x0;
    previousStatusWord = 0x0;
    currentState = CIA402_State::START;
    previousState = currentState;
  }
};

#endif  // CONTROLLER_HPP_