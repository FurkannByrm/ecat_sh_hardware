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


int main(int argc, char** argv)
{




}