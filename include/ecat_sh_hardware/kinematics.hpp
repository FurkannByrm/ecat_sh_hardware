/**
 * @file kinematics.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <chrono>
#include <bits/stl_pair.h>
#include <math.h>

using timepoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double, std::ratio<1>>>;

constexpr auto WHEEL_POSITION_INCREMENT = 16384.0;
constexpr auto WHEEL_RADIUS = 0.105;
constexpr auto WHEEL_SEPERATION = 0.457;
constexpr auto WHEEL_TO_MOTOR_REDUCTION = 24.685;

struct WheelParams
{
  double radius;
  double wheel_seperation;
  double gear_ratio;
  double increment;
};

struct Odometry
{
  double linearVel;   // [m/s]
  double angularVel;  // [rad/s]
  double x;           // [m]
  double y;           // [m]
  double heading;     // rad

  WheelParams wheelParams;

  timepoint previousUpdateTime;

  Odometry(){}

  Odometry& operator=(Odometry other)
  {
    this->linearVel = other.linearVel;
    this->angularVel = other.angularVel;
    this->x = other.x;
    this->y = other.y;
    this->heading = other.heading;
    return *this;
  }

  void reset();

  Odometry update(double left_wheel_vel, double right_wheel_vel, const timepoint update_time);
};

/**
 * @brief Get the Wheel Velocity From Robot Cmd object
 *
 * @param linear_vel
 * @param angular_vel
 * @return std::pair<double, double> return right and left wheel angular velocities respectively
 */
std::pair<double, double> getWheelVelocityFromRobotCmd(double linear_vel, double angular_vel);

inline const double motorPositionToJointPosition(const int32_t& motor_position)
{
  constexpr auto coe = (1.0 / WHEEL_POSITION_INCREMENT) * (2.0 * M_PI) / WHEEL_TO_MOTOR_REDUCTION;
  return ((double)motor_position) * coe;
}

/**
 * @brief
 *
 * @param joint_position
 * @return const int32_t
 */
inline const int32_t jointPositionToMotorPosition(const double& joint_position)
{
  constexpr auto coe = (WHEEL_POSITION_INCREMENT * WHEEL_TO_MOTOR_REDUCTION) / (2.0 / M_PI);
  return joint_position * coe;
}

/**
 * @brief Turns the joints current angular velocity, calculated from the motors RPM.
 *
 * @param motor_velocity RPM
 * @return const double angular velocity of the joint
 */
inline const double motorVelocityToJointVelocity(const int32_t& motor_velocity)
{
  constexpr auto coe = (2.0 * M_PI) / (60.0 * WHEEL_TO_MOTOR_REDUCTION);
  return (double(motor_velocity)) * coe;
}

/**
 * @brief
 *
 * @param joint_velocity
 * @return const int32_t
 */
inline const int32_t jointVelocityToMotorVelocity(const double& joint_velocity)
{
  constexpr auto coe = (60.0 * WHEEL_TO_MOTOR_REDUCTION) / (2.0 * M_PI);
  return joint_velocity * coe;
}

#endif  // KINEMATICS_HPP_