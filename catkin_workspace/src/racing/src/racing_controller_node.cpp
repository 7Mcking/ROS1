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
 * @file racing_controller_node.h
 */

#include "RacingController.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <memory>


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
RacingController *racing_controller;

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

  // Copy the distances to the buffer
  for(size_t i=0; i<3;i++){
    distances[i] = msg->ranges[i];
  }

  // Copy new distances into the controller buffer
  racing_controller->updateDistances(distances);
  // Calculate new velocity and steering angle
  racing_controller->calculateNewAction();
  // Copy new velocity and steering angle into the local buffer
  linear_velocity = racing_controller->getNewVelocity();
  steering_angle = racing_controller->getNewSteeringAngle();

  // Create a vector from the steering angle
  geometry_msgs::Vector3 steering;
  steering.z = steering_angle;

  // Create a vector from the linear velocity
  geometry_msgs::Vector3 velocity;
  velocity.x = linear_velocity;

  // Create a Twist from both vectors
  geometry_msgs::Twist new_action;
  new_action.linear = velocity;
  new_action.angular = steering;

  // Push the new Twist on to the topic
  publisher_actions->publish(new_action);
}

int main(int argc, char* argv[]) {
  // Initialise the new node
  ros::init(argc, argv, "vehicle_controller");
  ros::NodeHandle node_handle;
  ROS_INFO("Vehicle controller started.");

  // Set default read and write topics
  std::string subscribe_topic_sensors = "vehicle/laser";
  std::string publish_topic_actors = "vehicle/actors";

  // Get read and write targets from launch file parameter
  node_handle.param<std::string>("vehicle_controller/laser/topic",
                                 subscribe_topic_sensors,
                                 subscribe_topic_sensors);

  node_handle.param<std::string>("vehicle_controller/actors/topic",
                                 publish_topic_actors,
                                 publish_topic_actors);

  ROS_INFO("Vehicle controller will listen to: %s", subscribe_topic_sensors.c_str());
  ROS_INFO("Vehicle controller will write to: %s", publish_topic_actors.c_str());

  // Initiate controller, publisher and subscriber
  racing_controller = new RacingController;
  subscriber_sensor_data = new ros::Subscriber;
  publisher_actions = new ros::Publisher;

  // Define publisher and subscriber
  *subscriber_sensor_data = node_handle.subscribe(subscribe_topic_sensors, 10, callbackLaserSensor);
  *publisher_actions = node_handle.advertise<geometry_msgs::Twist>(publish_topic_actors, 10);

  // Prevent ros from termination
  ROS_INFO("Vehicle controller is running...");
  ros::spin();

  // Clean heap storage
  delete publisher_actions;
  delete subscriber_sensor_data;
  delete racing_controller;
  return 0;
}
