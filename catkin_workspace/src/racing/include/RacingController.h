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
 * @file RacingController.h
 */

/**
 * @brief Controller that will control the vehicle behaviour based on the sensor data.
 */
class RacingController {
 private:
  /**
   * @brief Current sensor states.
   * @details Ordered mathematical positive like left, front, right.
   */
  float sensor_distances_[3] = {0.0f, 0.0f, 0.0f};

  /**
   * @brief Target velocity based on the distance to the next target in front.
   */
  float target_velocity_ = 0.0f;

  /**
   * @brief Target streering angle based on the distance to the left and right.
   */
  float target_steering_angle_ = 0.0f;

  /**
   * @brief Will calculate the new target velocity based on the sensor data available.
   */
  void calculateTargetVelocity();

  /**
   * @brief Will calculate the new target steering angle based on the sensor data available.
   */
  void calculateTargetSteeringAngle();
 public:

  /**
   * @brief Will update the stored sensor data with new one.
   * @param distances Distances ordered mathematical positive like left, front, right.
   */
  void updateDistances(const float distances[3]);

  /**
   * @brief Will calculate velocity and steering angle based on the sensor data.
   */
  void calculateNewAction();

  /**
   * @brief Access to the velocity calculated based on the sensor data.
   * @return Target velocity
   */
  float getNewVelocity();

  /**
   * @brief Access to the steering angle calculated based on the sensor data.
   * @return Target steering angle
   */
  float getNewSteeringAngle();

};
