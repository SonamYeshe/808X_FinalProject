/*
 * Navigation.h
 *
 *  Created on: Dec 11, 2017
 *      Author: sonamyeshe
 */

#ifndef FINALPROJECT_INCLUDE_NAVIGATION_H_
#define FINALPROJECT_INCLUDE_NAVIGATION_H_
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "../include/finalproject/Frontier.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation {
 public:
  Navigation(ros::NodeHandle& n, tf::TransformListener& list);
  void frontierCallback(const sensor_msgs::PointCloud frontier_cloud);
 protected:
  int getNearestFrontier(const sensor_msgs::PointCloud frontierGoal,
                         const tf::StampedTransform transform);
  ros::Subscriber frontierSub;
  tf::TransformListener *tfListener;
};

#endif /* FINALPROJECT_INCLUDE_NAVIGATION_H_ */
