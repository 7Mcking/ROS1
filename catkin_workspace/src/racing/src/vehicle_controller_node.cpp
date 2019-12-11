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
 * @author Simon Schaefer, Michael Hoss
 * @file vehicle_controller_node.h
 */

#include "VehicleController.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <memory>

/* Declare variables outside of functions, such that they can be used
 * both in main, and in the callback function.
 */
ros::Publisher *publisher_actions = nullptr;
ros::Subscriber *subscriber_sensor_data = nullptr;
VehicleController *vehicle_controller = nullptr;

/**
 * @brief Callback function that is automatically triggered when a new Lidar scan is available
 * @param msg A pointer to message object that contains the new Lidar scan
 */
void callbackLaserSensor(const sensor_msgs::LaserScanPtr &msg) {

  ROS_INFO("Solution: log message on receipt of a new Lidar message.");
  ROS_INFO("Solution: Lidar distances: right=%2f, front=%2f, left=%2f.",
           msg->ranges[0],
           msg->ranges[1],
           msg->ranges[2]);

  // Copy argument data to local variable
  float distances[3] = {0.0f, 0.0f, 0.0f};
  for(size_t i=0; i<3;i++){
    distances[i] = msg->ranges[i];
  }

  // Interface calls to the VehicleController instance
  vehicle_controller->overwriteLidarDistances(distances);
  vehicle_controller->computeTargetValues();
  double linear_velocity = vehicle_controller->getTargetVelocity();
  double steering_angle = vehicle_controller->getTargetSteeringAngle();

  // Convert local variables to a geometry_msgs::Twist message for publishing.
  geometry_msgs::Twist new_action;
  geometry_msgs::Vector3 steering;
  geometry_msgs::Vector3 velocity;
  steering.z = steering_angle;
  velocity.x = linear_velocity;
  new_action.linear = velocity;
  new_action.angular = steering;

  // Publish the newly computed actuator command to the topic
  publisher_actions->publish(new_action);
}

int main(int argc, char* argv[]) {
  // Initialize the ROS node
  ros::init(argc, argv, "vehicle_controller");
  ros::NodeHandle node_handle;
  ROS_INFO("Vehicle controller started.");

  // Solution: hard-code the right topic names
  std::string subscribe_topic_sensors = "vehicle/lidar_measurements";
  std::string publish_topic_actuators = "vehicle/actuator_commands";

  ROS_INFO("Vehicle controller subscribes to: %s", subscribe_topic_sensors.c_str());
  ROS_INFO("Vehicle controller publishes to: %s", publish_topic_actuators.c_str());

  // Initialize / allocate dynamic memory
  vehicle_controller = new VehicleController;
  subscriber_sensor_data = new ros::Subscriber;
  publisher_actions = new ros::Publisher;

  // Connect subscriber and publisher to their respective topics and callback function
  *subscriber_sensor_data = node_handle.subscribe(subscribe_topic_sensors, 10, callbackLaserSensor);
  *publisher_actions = node_handle.advertise<geometry_msgs::Twist>(publish_topic_actuators, 10);

  // Solution: Adapt the event loop such that incoming messages are looked for exactly every 20ms
  ros::Rate loop_rate(50); // 20ms --> 50Hz
  ROS_INFO("Vehicle controller is running...");
  while (ros::ok())
  {
    ROS_INFO("Solution: log message before scanning subscribed topic for new messages");
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Clean dynamic memory
  delete publisher_actions;
  delete subscriber_sensor_data;
  delete vehicle_controller;
  return 0;
}
