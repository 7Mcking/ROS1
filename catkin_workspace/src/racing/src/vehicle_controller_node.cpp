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
 * @date 18.08.19
 * @file vehicle_controller_node.h
 */

#include "VehicleController.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <memory>

/***************************************************************
  All sections that are marked with a [TODO] rag are missing up
  to four lines of code.
****************************************************************/

/**
 * @brief Publisher for the target actions.
 */
ros::Publisher *publisher_actions = nullptr;
/**
 * @brief Subscriber for the sensor data.
 */
ros::Subscriber *subscriber_sensor_data = nullptr;

/**
 * @brief Pointer to the vehicle controller.
 */
VehicleController *vehicle_controller;

/**
 * @brief Callback function that is triggered if a new set of points is available.
 * @param msg A pointer to message of type LaserScan that contains the new data.
 */
void callbackLaserSensor(const sensor_msgs::LaserScanPtr &msg) {
  // Buffer for the new distances
  float distances[3] = {0.0f, 0.0f, 0.0f};
  // Buffer for the new linear velocity
  float linear_velocity = 0.0f;
  // Buffer for the new steering angle
  float steering_angle = 0.0f;

  // [TODO] Extract the ranges from the laser scan and put the into distance buffer

  // [TODO] Copy the distances into the controller and caluclate the new actions

  // [TODO] Extract the new velocity and steering angle from the controller

  // Create a vector from the steering angle
  geometry_msgs::Vector3 steering;
  // [TODO] Convert the steering angle to a vector

  // Create a vector from the linear velocity
  geometry_msgs::Vector3 velocity;
  // [TODO] Convert the velocity to a vector

  // Create a Twist from both vectors
  geometry_msgs::Twist new_action;
  // [TODO] Set you linear and angular velocity vectors into the twist message

  // Push the new Twist on to the topic
  publisher_actions->publish(new_action);
}

int main(int argc, char* argv[]) {
  // [TODO] Init your node here 

  
  // Set default read and write topics
  std::string default_subscribe_topic_sensors = "";
  std::string default_publish_topic_actors = "";

  // BONUS [TODO] Read parameter from the launch file to set the topics

  // Initiate controller, publisher and subscriber
  vehicle_controller = new VehicleController;
  subscriber_sensor_data = new ros::Subscriber;
  publisher_actions = new ros::Publisher;

  // [TODO] Allocate new space for the subscriber and the publisher


  // [TODO] Create subscriber and publisher for the allocated space


  // [TODO] Prevent the node from termination by spinning ROS

  return 0;
}
