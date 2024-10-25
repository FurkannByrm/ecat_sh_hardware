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

#include <algorithm>

struct VelocityLimiter
{
  double max_vel;
  double min_vel;
  double max_acc;
  double min_acc;
  double max_jerk;
  double min_jerk;
  double previousValue;
  double ppreviousValue;

  VelocityLimiter();

  void limit(double& command, double current_value, double dt);
};

#endif // CONTROLLER_HPP_