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

void Navigation::frontierCallback(
    const sensor_msgs::PointCloud frontierGoal) {
  tf::StampedTransform transform;
  tfListener->waitForTransform("/map", "/odom", ros::Time(0),
                               ros::Duration(3.0));
  tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
  if (frontierGoal.points.size() == 0) {
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        0, 0, 3.14);
    ac.sendGoal(goal);
    ac.waitForResult();
  } else {
    int nearestGoalNum = Navigation::getNearestFrontier(frontierGoal,
                                                        transform);
    ROS_INFO("Closest frontier: %d", nearestGoalNum);
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = frontierGoal.points[nearestGoalNum].x;
    goal.target_pose.pose.position.y = frontierGoal.points[nearestGoalNum].y;
    goal.target_pose.pose.position.z = frontierGoal.points[nearestGoalNum].z;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        0, 0, 0);
    ROS_INFO("Sending goal......Navigating to: x: %f y: %f",
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(45.0));
    ROS_INFO("move_base goal published");
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base has moved to %f,%f",
               goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      goal.target_pose.pose.orientation =
          tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14);
      ac.sendGoal(goal);
      ac.waitForResult();
    } else {
      ROS_INFO("The base fail to move!");
    }
  }
}
/*
 * compare then select the median point with minimal distance to the turtlebot.
 */
int Navigation::getNearestFrontier(const sensor_msgs::PointCloud frontierGoal,
                                   const tf::StampedTransform transform) {
  int nearestGoalNum;
  float shortestLength = 100000000000;
  for (int i = 0; i < frontierGoal.points.size(); ++i) {
    float length = sqrt(
        (float(frontierGoal.points[i].x - transform.getOrigin().x()))
            ^ 2 + (float(frontierGoal.points[i].y - transform.getOrigin().y()))
            ^ 2);
    if (length < shortestLength) {
      shortestLength = length;
      nearestGoalNum = i;
    }
  }
  return nearestGoalNum;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebotNavigation");
  ros::NodeHandle n;
  tf::TransformListener listener;
  Navigation autoNavigator;
  ros::Subscriber autoNavigator_sub = n.subscribe("/frontierPossibilities", 1,
                                                  &Navigation::frontierCallback,
                                                  &autoNavigator);
  ROS_INFO("INFO! Navigation Started");
  ros::Rate loop_rate(30);
  while (ros::ok() && n.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
