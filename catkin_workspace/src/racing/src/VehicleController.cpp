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

void VehicleController::calculateTargetVelocity() {
  const float FACTOR = 2.0f;
  const float FACTOR2 = 8.0f;
  // Use a parameterized response curve
  this->target_velocity_ = this->sensor_distances_[1]*std::tanh(this->sensor_distances_[1]*FACTOR2)*FACTOR;
}

void VehicleController::calculateTargetSteeringAngle() {
  const float FACTOR = 0.2f;
  const float FACTOR2 = 0.4f;
  // Calcualte relative position on road
  float  diff = this->sensor_distances_[2]-this->sensor_distances_[0];
  // Use a parameterized response curve
  this->target_steering_angle_ = 1.0/(FACTOR2*this->sensor_distances_[1])*(diff)*std::tanh(std::abs(diff))*FACTOR;
}

void VehicleController::updateDistances(const float distances[3]) {
  // Copy to local buffer
  // Maybe a std::memcpy is more efficiently here
  for(size_t i = 0; i<3; i++){
    this->sensor_distances_[i] = distances[i];
  }
}

void VehicleController::calculateNewAction() {
  // Calculate new velocity
  calculateTargetVelocity();
  // Calculate new steeting angle
  calculateTargetSteeringAngle();
}

float VehicleController::getNewVelocity() {
  return this->target_velocity_;
}

float VehicleController::getNewSteeringAngle() {
  return this->target_steering_angle_;
}





