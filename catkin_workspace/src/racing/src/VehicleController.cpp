/*    Modified by Institute for Automotive Engineering (ika), RWTH University
 *    All rigths reserved
 *
 *    This file is part of "Self-Driving Lab I - Software Framework".
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted exclusively under the terms defined in
 *    the licese file. You should have received a copy of the license with
 *    this file. If not, please visit:
 *    https://git.rwth-aachen.de/ika/sdl1-ws2019.
 */
/**
 * @author Simon Schaefer
 * @date 10.08.19
 * @file VehicleController.cpp
 */

#include "VehicleController.h"

#include <cstdlib>
#include <cmath>

void VehicleController::computeTargetVelocity() {
  const float FACTOR = 2.0f;
  const float FACTOR2 = 8.0f;
  const float& front_distance = this->sensor_distances_[1];

  // The larger the free space in the front, the greater the velocity can be.
  // The tanh function - tuned with parameters - is useful to implement such behavior smoothly.
  this->target_velocity_ = front_distance * FACTOR * std::tanh(front_distance * FACTOR2);
}

void VehicleController::computeTargetSteeringAngle() {
  const float FACTOR = 0.5f;

  // Calcualte relative lateral position on the road
  // (how far towards right border is the cart located)
  const float& right_distance = this->sensor_distances_[0];
  const float& front_distance = this->sensor_distances_[1];
  const float& left_distance = this->sensor_distances_[2];
  float rightShift = left_distance - right_distance;

  // The closer the cart is to the right border, the more it should steer to the left
  // (and vice versa).
  // (Steering to the left = positive steering angle)
  // To achieve this in a smooth way, a parameter-tuned tanh function can be used.
  this->target_steering_angle_ = (FACTOR/front_distance) * rightShift * std::tanh( std::abs(rightShift));
}

void VehicleController::overwriteLidarDistances(const float distances[3]) {
  for(size_t i = 0; i<3; i++){
    this->sensor_distances_[i] = distances[i];
  }
}

void VehicleController::computeTargetValues() {
  computeTargetVelocity();
  computeTargetSteeringAngle();
}

double VehicleController::getTargetVelocity() {
  return this->target_velocity_;
}

double VehicleController::getTargetSteeringAngle() {
  return this->target_steering_angle_;
}





