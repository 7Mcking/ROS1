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
  this->target_velocity_ = 2.0; // TODO: USE SENSOR DATA TO COMPUTE THIS
}

void VehicleController::computeTargetSteeringAngle() {
  this->target_steering_angle_ = 0.0; // TODO: USE SENSOR DATA TO COMPUTE THIS
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





