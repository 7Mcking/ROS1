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

/***************************************************************
  All sections that are marked with a [TODO] rag are missing up
  to four lines of code.
****************************************************************/

void VehicleController::calculateTargetVelocity() {
  // [TODO] Add your code to control the velocity here
}

void VehicleController::calculateTargetSteeringAngle() {
  // [TODO] Add your code to control the steering angle here
}

void VehicleController::updateDistances(const float distances[3]) {
  // [TODO] Copy the new distances in to the buffer provided by this class
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





