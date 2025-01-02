/**
 * @file kinematics.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "ecat_sh_hardware/kinematics.hpp"

Odometry Odometry::update(double left_wheel_vel, double right_wheel_vel, const timepoint update_time)
{

  const double linearVelBody = 0.5 * (left_wheel_vel + right_wheel_vel);
  const double angularVelBody = (right_wheel_vel - left_wheel_vel) / 0.45;
  /* std::cout << "Lin: " << linearVelBody << " ang; " << angularVelBody << std::endl; */
  const double dt = (update_time - previousUpdateTime).count();
  previousUpdateTime = update_time;
  if(angularVelBody <= 0.0001) // 2nd order Runge-Kutta integration:
  {
    
    double changeInHeading = angularVelBody * dt;
    x += linearVelBody * dt * std::cos(heading + (changeInHeading / 2.0));
    y += linearVelBody * dt * std::sin(heading + (changeInHeading / 2.0));
    heading += changeInHeading;
  }
  else{ // Exact Integration
    const double linearVelToAngularVelRatio = linearVelBody / angularVelBody;
    const double oldHeading = heading;
    heading += angularVelBody * dt;
    x += linearVelToAngularVelRatio * (std::sin(heading) - std::sin(oldHeading));
    y += linearVelToAngularVelRatio * (std::cos(heading) - std::cos(oldHeading));
  }

  // TODO: Use Rolling Window Accumulation for velocities??
  
  linearVelAccumulator(linearVelBody / dt);
  angularVelAccumulator(angularVelBody / dt);

  linearVel = boost::accumulators::rolling_mean(linearVelAccumulator);
  angularVel = boost::accumulators::rolling_mean(angularVelAccumulator);

  return *this;

}

std::pair<double, double> getWheelVelocityFromRobotCmd(double linear_vel, double angular_vel)
{
  double rightWheelVel = (linear_vel + (angular_vel * (WHEEL_SEPERATION / 2.0))) / WHEEL_RADIUS;
  double leftWheelVel = (linear_vel - (angular_vel * (WHEEL_SEPERATION / 2.0))) / WHEEL_RADIUS;
  return std::make_pair(rightWheelVel, leftWheelVel);
}

void Odometry::reset()
{
  linearVel = 0.0;
  angularVel = 0.0;
  x = 0.0;
  y = 0.0;
  heading = 0.0;
}