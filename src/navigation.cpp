/**
 *  @file       navigation.cpp
 *  @brief      deduct closest frontier edge and ask turtlebot move to it's position
 *  @details    subscribe to the /frontierPossibilities topic to get all possible
 *  frontiers, select the optimal one with minimal distance to the turtlebot.
 *  send a simple_navigation goal to ask turtlebot move to there. turtle bot should
 *  first rotate 360 degrees to update original map.
 *  @author     Jiawei Ge(SonamYeshe)
 *  @copyright  BSD, GNU Public License 2017 Jiawei Ge
 *  @disclaimer Jiawei Ge(SonamYeshe), hereby disclaims all copyright interest
 *  in the program `finalproject' (which makes passes at compilers) written by
 *  Jiawei Ge(SonamYeshe).
 <signature of Jiawei Ge>, 15 December 2017
 Jiawei Ge
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "../include/finalproject/Navigation.h"

/**
 *  @brief  acquire data from /frontierPossibilities topic and find the goal
 *  position in the map. move turtlebot to that position finally.
 *  @param  const msg type sensor_msgs::PointCloud
 */
void Navigation::frontierCallback(
    const sensor_msgs::PointCloud frontierGoal) {
  /*
   * listen to tf from /map to /odom to prepare turtlebot position conscience
   */
  tf::StampedTransform transform;
  tfListener->waitForTransform("/map", "/odom", ros::Time(0),
                               ros::Duration(3.0));
  tfListener->lookupTransform("/map", "/odom", ros::Time(0), transform);
  if (frontierGoal.points.size() == 0)
    return;
  /*
   * calculate the frontier cell with minimal distance to the turtlebot and
   * using move_base to move turtlebot there.
   */
  int nearestGoalNum = Navigation::getNearestFrontier(frontierGoal, transform);
  ROS_INFO("Closest frontier: %d", nearestGoalNum);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


  goal.target_pose.pose.position.x = frontierGoal.points[nearestGoalNum].x;
  goal.target_pose.pose.position.y = frontierGoal.points[nearestGoalNum].y;
  goal.target_pose.pose.position.z = frontierGoal.points[nearestGoalNum].z;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      0, 0, 1);
  ROS_INFO("Sending goal......Navigating to: x: %f y: %f",
           goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(30.0));
  ROS_INFO("move_base goal published");
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base has moved to %f,%f",
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
  } else {
    ROS_INFO("Update the goal. Frontiers has been refreshed.");
  }
}
/*
 *  @brief  compare then select the median point with minimal distance to the turtlebot.
 *  @param  const msg type sensor_msgs::PointCloud
 *  @param  const tf::StampedTransform
 */

int Navigation::getNearestFrontier(const sensor_msgs::PointCloud frontierGoal,
                                   const tf::StampedTransform transform) {
  int nearestGoalNum;
  float shortestLength = 1000000000;
  for (int i = 0; i < frontierGoal.points.size(); ++i) {
    float length = sqrt(
        pow((float(frontierGoal.points[i].x - transform.getOrigin().x())), 2.0)
            + pow((float(frontierGoal.points[i].y - transform.getOrigin().y())),
                  2.0));
    if (length > 1.25 && length < shortestLength) {
      shortestLength = length;
      nearestGoalNum = i;
    }
  }
  return nearestGoalNum;
}
/**
 *  @brief  acquire frontier possibilities and ask turtlebot move to the closet one.
 *  @param  integer of argument count
 *  @param  char pointer of argument value
 *  @return a bool value
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebotNavigator");
  ros::NodeHandle n;
  tf::TransformListener listener;
  Navigation autoNavigator;
  autoNavigator.tfListener = &listener;
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
