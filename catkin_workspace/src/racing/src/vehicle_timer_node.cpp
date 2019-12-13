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
 * @date 11.12.2019
 * @file vehicle_timer_node.h
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <memory>

ros::Subscriber *subscriber_position_data = nullptr;
bool quadrants[4] = {false, false, false, false};
ros::Time *time_buffer = nullptr;

void callbackPosition(const nav_msgs::OdometryPtr &msg) {
  double& x = msg->pose.pose.position.x;
  double& y = msg->pose.pose.position.y;

  if (x > 0 && y < 0 ) {
    quadrants[0] = true;
  } else if (x < 0 && y < 0 && quadrants[0]) {
    quadrants[1] = true;
  } else if (x < 0 && y > 0 && quadrants[1]) {
    quadrants[2] = true;
  } else if (x > 0 && y > 0 && quadrants[2]) {
    quadrants[3] = true;
  }

  if (quadrants[3] && x > 0 && y < 0) {
    for (bool & quadrant : quadrants) {
      quadrant = false;
    }
    if(time_buffer){
      ros::Duration diff = ros::Time::now() - *time_buffer;
      boost::posix_time::time_duration my_posix_time = diff.toBoost();
      std::string iso_time_str = boost::posix_time::to_iso_string(my_posix_time);
      ROS_INFO("Lap-Time: %ss", iso_time_str.c_str());
    } else{
      time_buffer = new ros::Time;
    }
    ROS_INFO("Lap gets measured!.");
    *time_buffer = ros::Time::now();
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vehicle_timer");
  ros::NodeHandle node_handle;
  ROS_INFO("Vehicle timer started.");

  std::string default_subscribe_topic_position = "";
  std::string subscribe_topic_position;
  node_handle.param<std::string>("vehicle/position_topic",
                                 subscribe_topic_position,
                                 default_subscribe_topic_position);
  ROS_INFO("Vehicle timer subscribes to: %s", subscribe_topic_position.c_str());

  subscriber_position_data = new ros::Subscriber;
  *subscriber_position_data = node_handle.subscribe(subscribe_topic_position, 10, callbackPosition);

  ROS_INFO("Vehicle timer is running...");
  ros::spin();

  delete subscriber_position_data;
  return 0;
}
