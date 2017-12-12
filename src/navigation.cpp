/*
 * navigation.cpp
 *
 *  Created on: Dec 11, 2017
 *      Author: sonamyeshe
 */
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "../include/Navigation.h"
#include "../include/finalproject/Frontier.h"

Navigation::Navigation(ros::NodeHandle& n, tf::TransformListener& list) {
  tfListener = &list;
  frontierSub = n.subscribe("frontiers", 1, &Navigation::frontierCallback,
                             this);
}

void frontierCallback(const sensor_msgs::PointCloud frontier_cloud) {
  tf::StampedTransform transform;
  tfListener->waitForTransform("/map", "/odom", ros::Time(0),
                               ros::Duration(3.0));
  tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
  if (frontier_cloud.points.size() == 0)
    return;
//TODO:retrieve the frontier target
  ROS_INFO("Closest frontier: %d", frontier_i);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  bool at_target = false;
  int attempts = 0;
  while (!at_target && attempts < 2) {
    if (attempts >= 0) {
      frontier_i = (rand() % frontier_cloud.points.size());
      at_target = false;
    }
    attempts++;
    goal.target_pose.pose.position.x = frontier_cloud.points[frontier_i].x;
    goal.target_pose.pose.position.y = frontier_cloud.points[frontier_i].y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    goal.target_pose.pose.orientation = odom_quat;
    ROS_INFO("Navigating to: x: %f y: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(
        "move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server active");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(45.0));
    ROS_INFO("move_base goal published");
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      at_target = true;
      ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
          3.14);
      goal.target_pose.pose.orientation = odom_quat;
      ac.sendGoal(goal);
      ac.waitForResult();
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebotNavigation");
  ros::NodeHandle n;
  tf::TransformListener listener;
  Frontier frontierGoalCell;
  ros::Publisher = n.advertise<sensor_msgs::PointCloud>("/frontierGoalCell", 1);
  ros::Subscriber = n.subscribe("/map", 1, &Frontier::frontierTarget,
                                &frontierGoalCell);
  sensor_msgs::PointCloud::header.frame_id = "/map";
  /*
   * object the class Navigation
   */
  Navigation walker(n, listener);
  ROS_INFO("INFO! Navigation Started");
  ros::Rate loop_rate(30);
  while (ros::ok() && n.ok()) {
    frontier_pub.publish(frontier_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
