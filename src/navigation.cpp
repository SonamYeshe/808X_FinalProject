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
  /*
   * object the class Navigation
   */
  Navigation walker(n, listener);
  ROS_INFO("INFO! Navigation Started");
  ros::Rate loop_rate(30);
  while (ros::ok() && n.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/*
 * get turtlebot position in the map.
 */
/*
 int turtlebot_position = round(
 -map->info.origin.position.y / map->info.resolution) * map_width
 + round(map->info.origin.position.x / map->info.resolution);
 /*
 * compare then select the median point with minimal distance to the turtlebot.
 */
/*
 double shortestLength = 1000000000000000;
 int finalTarget;
 for (int i = 0; i < median.size(); ++i) {
 std::map<int, int> x;
 std::map<int, int> y;
 x[0] = median[i] % map_width;
 x[1] = turtlebot_position % map_width;
 y[0] = median[i] / map_width;
 y[1] = turtlebot_position / map_width;
 double length = sqrt((double(x[1] - x[0])) ^ 2 + (double(y[1] - y[0])) ^ 2);
 if (shortestLength > length) {
 shortestLength = length;
 finalTarget = i;
 }
 }
 int frontierTarget = median[finalTarget];
 frontierGoal.points.resize(1);
 frontierGoal.points[0].x = frontierTarget % map_width;
 frontierGoal.points[0].y = frontierTarget / map_width;
 frontierGoal.points[0].z = 0;
 */
