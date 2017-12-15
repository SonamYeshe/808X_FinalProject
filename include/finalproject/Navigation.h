/**
 *  @file       Navigation.h
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

#ifndef FINALPROJECT_INCLUDE_NAVIGATION_H_
#define FINALPROJECT_INCLUDE_NAVIGATION_H_
#include "ros/ros.h"
#include "ros/console.h"
#include <cstdlib>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation {
 public:

  /**
   *  @brief  acquire data from /frontierPossibilities topic and find the goal
   *  position in the map. move turtlebot to that position finally.
   *  @param  const msg type sensor_msgs::PointCloud
   */
  void frontierCallback(const sensor_msgs::PointCloud frontier_cloud);

  /*
   *  @brief  compare then select the median point with minimal distance to the turtlebot.
   *  @param  const msg type sensor_msgs::PointCloud
   *  @param  const tf::StampedTransform
   */
  int getNearestFrontier(
      const sensor_msgs::PointCloud frontierGoal,
                         const tf::StampedTransform transform);
  tf::TransformListener *tfListener;
};

#endif /* FINALPROJECT_INCLUDE_NAVIGATION_H_ */
