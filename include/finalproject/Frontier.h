/**
 *  @file       Frontier.h
 *  @brief      deduct some goal possibilities where the turtlebot should go
 *  @details    when given a map, find the available frontier edges (with width
 *  larger than the diameter of the turtlebot) and publish them with topic type
 *  std_msgs::PointCloud
 *  @author     Jiawei Ge(SonamYeshe)
 *  @copyright  BSD, GNU Public License 2017 Jiawei Ge
 *  @disclaimer Jiawei Ge(SonamYeshe), hereby disclaims all copyright interest
 *  in the program `finalproject' (which makes passes at compilers) written by
 *  Jiawei Ge(SonamYeshe).
 <signature of Jiawei Ge>, 15 December 2017
 Jiawei Ge
 */

#ifndef FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_
#define FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"

class Frontier {
 public:

  /**
   *  @brief  acquire data from /map topic and find the goal position in the map.
   *  @param  const pointer to msg type nav_msgs/OccupancyGrid
   *  @return goal position in the map.
   */
  void frontierTarget(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /**
   *  @brief  acquire data from /scan topic and a point in the map to
   *  distinguish is it's a frontier cell
   *  @param  const pointer to msg type nav_msgs/OccupancyGrid
   *  @param  integer of a point in the map
   *  @return void
   */
  bool isFrontierEdgeCell(const nav_msgs::OccupancyGrid::ConstPtr& map,
                          int position_num);

  /**
   *  @brief  deduct 8 neibors of a point in a map
   *  @param  const pointer to msg type nav_msgs/OccupancyGrid
   *  @param  integer of a point in the map
   *  @return void
   */
  void getNeibors(int neibors[], int position_num,
                  int map_width);
  static sensor_msgs::PointCloud frontierGoal;
};

#endif /* FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_ */
