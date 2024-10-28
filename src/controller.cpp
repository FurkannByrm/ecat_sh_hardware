/**
 * @file controller.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ecat_sh_hardware/controller.hpp"
#include "ecat_sh_hardware/shared_memory_handler.hpp"

VelocityLimiter::VelocityLimiter()
{
}

void VelocityLimiter::limit(double& command, double current_value, double dt)
{
  // Limit jerk
  // Limit acceleration

  const double requestedAcc = (current_value - command);
  const double possibleAcceleration = std::clamp(requestedAcc, min_acc * dt, max_acc * dt);
  command += possibleAcceleration;
  // Limit velocity

  command = std::clamp(command, min_vel, max_vel);
}

CIA402_State deriveState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000)
  {
    return CIA402_State::NOT_READY_TO_SWITCH_ON;
  }
  else if ((status_word & 0b01001111) == 0b01000000)
  {
    return CIA402_State::SWITCH_ON_DISABLED;
  }
  else if ((status_word & 0b01101111) == 0b00100001)
  {
    return CIA402_State::READY_TO_SWITCH_ON;
  }
  else if ((status_word & 0b01101111) == 0b00100011)
  {
    return CIA402_State::SWITCH_ON;
  }
  else if ((status_word & 0b01101111) == 0b00100111)
  {
    return CIA402_State::OPERATION_ENABLED;
  }
  else if ((status_word & 0b01101111) == 0b00000111)
  {
    return CIA402_State::QUICK_STOP_ACTIVE;
  }
  else if ((status_word & 0b01001111) == 0b00001111)
  {
    return CIA402_State::FAULT_REACTION_ACTIVE;
  }
  else if ((status_word & 0b01001111) == 0b00001000)
  {
    return CIA402_State::FAULT;
  }
  return CIA402_State::UNKNOWN_STATE;
}

uint16_t transitionToState(CIA402_State state, uint16_t control_word)
{
  switch (state)
  {
    case CIA402_State::START:
      return control_word;
    case CIA402_State::NOT_READY_TO_SWITCH_ON:
      return control_word;
    case CIA402_State::SWITCH_ON_DISABLED:
      return (control_word & 0x01111110) | 0x00000110;
    case CIA402_State::READY_TO_SWITCH_ON:
      return (control_word & 0b01110111) | 0b00000111;
    case CIA402_State::SWITCH_ON:
      return (control_word & 0b01111111) | 0b00001111;
    case CIA402_State::OPERATION_ENABLED:
      return control_word;
    case CIA402_State::QUICK_STOP_ACTIVE:
      return (control_word & 0b01111111) | 0b00001111;
    case CIA402_State::FAULT_REACTION_ACTIVE:
      return control_word;
    case CIA402_State::FAULT:
      return (control_word & 0b11111111) | 0b10000000;
    default:
      break;
  }
  return control_word;
}

int main(int argc, char** argv)
{
  SharedMemoryHandler shMemHandler;

  if (shMemHandler.init(ShMode::User) != ecat_sh_hardware::Error::NoError)
  {
    return 1;
  }

  bool run = true;

  shared_obj_info::EthercatDataObject rightWheelData;
  DriverStateHandler rightWheelStateHandler;
  shared_obj_info::EthercatDataObject leftWheelData;
  DriverStateHandler leftWheelStateHandler;

  while (run)
  {
    shMemHandler.lockSem();

    auto dataPackageOpt = shMemHandler.getEcDataObject();
    if (dataPackageOpt)
    {
      const std::vector<shared_obj_info::EthercatDataObject> dataPackage = dataPackageOpt.value();
      if (!dataPackage.empty())
      {
        // dataPackage[0] := right wheel
        rightWheelData = dataPackage[0];

        if (CIA402_State rightWheelCurrentState = deriveState(rightWheelData.status_word);
            rightWheelCurrentState != CIA402_State::OPERATION_ENABLED)
        {
          uint16_t newControlWord = transitionToState(rightWheelCurrentState, rightWheelData.control_word);
          rightWheelData.control_word = newControlWord;
          rightWheelStateHandler.previousControlWord = newControlWord;
          rightWheelStateHandler.isOperational = false;
        }
        else
        {
          rightWheelStateHandler.isOperational = true;
        }

        // dataPackage[1] := left wheel
        leftWheelData = dataPackage[0];
        if (CIA402_State leftWheelCurrentState = deriveState(leftWheelData.status_word);
            leftWheelCurrentState != CIA402_State::OPERATION_ENABLED)
        {
          uint16_t newControlWord = transitionToState(leftWheelCurrentState, leftWheelData.control_word);
          leftWheelData.control_word = newControlWord;
          leftWheelStateHandler.previousControlWord = newControlWord;
          leftWheelStateHandler.isOperational = false;
        }
        else
        {
          leftWheelStateHandler.isOperational = true;
        }
      }

      
    }

    bool driversEnabled = (rightWheelStateHandler.isOperational && leftWheelStateHandler.isOperational);

    // If all slaves are operational, write commands:
    if (driversEnabled)
    {

    }

    shMemHandler.sendEcDataObject({ rightWheelData, leftWheelData });

    shMemHandler.freeSem();
  }
}