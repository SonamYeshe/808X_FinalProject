/*
 * Frontier.h
 *
 *  Created on: Dec 10, 2017
 *      Author: sonamyeshe
 */

#ifndef FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_
#define FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_
#include <vector>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class Frontier {
 public:
  const int map_width;
  int frontierTarget(const nav_msgs::OccupancyGrid::ConstPtr& map);
  bool isFrontierEdgeCell(const nav_msgs::OccupancyGrid::ConstPtr& map,
                          int position_num);
  void getNeibors(int neibors[], int position_num);
};

#endif /* FINALPROJECT_INCLUDE_FINALPROJECT_FRONTIER_H_ */
