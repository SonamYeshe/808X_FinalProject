/**
 *  @file       frontier.cpp
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

#include <vector>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include "../include/finalproject/Frontier.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"

sensor_msgs::PointCloud Frontier::frontierGoal;
/**
 *  @brief  acquire data from /map topic and find the goal position in the map.
 *  @param  const pointer to msg type nav_msgs/OccupancyGrid
 *  @return goal position in the map.
 */
void Frontier::frontierTarget(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  int map_width = map->info.width;
  std::vector<int> frontierEdgeCell;
  /*
   * find all edge possibilities.
   */
  for (int i = 0; i < map->data.size(); ++i) {
    if (isFrontierEdgeCell(map, i)) {
      frontierEdgeCell.push_back(i);
    }
  }
  /*
   * check every possibilities of connected cells and add them to different Open list,
   * erase one position in an Open list after using it finding all connected edge cells.
   * also erase it from frontierEdgeCell.
   */
  std::vector<int> frontierEdgeOpen;
  std::map<int, std::vector<int> > frontierEdgeClose;
  int i = 0;
  while (frontierEdgeCell.size() > 0) {
    frontierEdgeOpen.push_back(frontierEdgeCell.back());
    while (frontierEdgeOpen.size() != 0) {
      int neibors[8];
      int frontierCellUsed = frontierEdgeOpen.front();
      getNeibors(neibors, frontierCellUsed, map_width);
      for (int a = 0; a < 8; ++a) {
        for (int j = frontierEdgeCell.size() - 1; j >= 0; --j) {
          if (neibors[a] == frontierEdgeCell[j]) {
            int kcount = 0;
            for (int k = 0; k < frontierEdgeOpen.size(); ++k) {
              if (frontierEdgeOpen[k] != neibors[a]) {
                kcount += 1;
              }
              if (kcount == frontierEdgeOpen.size()) {
                if (frontierEdgeClose[i].size() != 0) {
                  int lcount = 0;
                  for (int l = 0; l < frontierEdgeClose[i].size(); ++l) {
                    if (frontierEdgeClose[i][l] != neibors[a]) {
                      lcount += 1;
                    }
                    if (lcount == frontierEdgeClose[i].size()) {
                      frontierEdgeOpen.push_back(neibors[a]);
                    }
                  }
                } else {
                  frontierEdgeOpen.push_back(neibors[a]);
                }
              }
            }
          }
        }
      }
      frontierEdgeClose[i].push_back(frontierCellUsed);
      frontierEdgeOpen.erase(frontierEdgeOpen.begin());
    }
    /*
     * delete all points in the frontierEdgeCell after adding them
     * into the frontierEdgeClose.
     */
    for (int j = 0; j < frontierEdgeClose[i].size(); ++j) {
      for (int k = 0; k < frontierEdgeCell.size(); ++k) {
        if (frontierEdgeClose[i][j] == frontierEdgeCell[k]) {
          frontierEdgeCell.erase(frontierEdgeCell.begin() + k);
        }
      }
    }
    i += 1;
  }
  /*
   * OLD: find median position of all the edges longer than 0.1m, which is a
   * little bigger than turtlebot's diameter.
   * NEW: sensor is not accurate. it put some 100 values between unknown frontiers.
   * DON't treat it serious! define 2 frontier cells in a row as the available frontier.
   */
  std::vector<int> median;
  for (int i = 0; i < frontierEdgeClose.size(); ++i) {
    /*
    std::map<int, int> x;
    std::map<int, int> y;
    x[0] = frontierEdgeClose[i].front() % map->info.width;
    x[1] = frontierEdgeClose[i].back() % map->info.width;
    y[0] = frontierEdgeClose[i].front() / map->info.width;
    y[1] = frontierEdgeClose[i].back() / map->info.width;
    float edgeLength = sqrt(float((x[1] - x[0]) ^ 2 + (y[1] - y[0]) ^ 2))
        * map->info.resolution;
     if (edgeLength > 0.01) {
      median.push_back(frontierEdgeClose[i][frontierEdgeClose[i].size() / 2]);
    }
     */
    if (frontierEdgeClose[i].size() > 1) {
      median.push_back(frontierEdgeClose[i][frontierEdgeClose[i].size() / 2]);
    }
  }
  /*
   * set frontier possible values in frontierGoal msg
   */
  Frontier::frontierGoal.points.resize(median.size());
  for (int i = 0; i < median.size(); ++i) {
    frontierGoal.points[i].x = float(
        ((median[i] % map->info.width)
            + map->info.origin.position.x / map->info.resolution)
            * map->info.resolution);
    frontierGoal.points[i].y = float(
        ((median[i] / map->info.width)
            + map->info.origin.position.y / map->info.resolution)
            * map->info.resolution);
    frontierGoal.points[i].z = float(0);
  }
  ROS_INFO("The positions of frontier median points have been published!");
}

/**
 *  @brief  acquire data from /scan topic and a point in the map to
 *  distinguish is it's a frontier cell
 *  @param  const pointer to msg type nav_msgs/OccupancyGrid
 *  @param  integer of a point in the map
 *  @return void
 */
bool Frontier::isFrontierEdgeCell(const nav_msgs::OccupancyGrid::ConstPtr& map,
                                  int position_num) {
  /**
   * define frontier edge cell to "be Any open cell adjacent to an unknown cell".
   * Reference: B. Yamauchi, “A frontier based approach for autonomous exploration,”
   *  in IEEE Int. Symp. Computational Intelligence in Robotics and Automa-tion,
   *  Monterey, CA, Jul., 10–11 1997, p. 146.
   */
  if (map->data[position_num] != 0) {
    return false;
  } else {
    int neibors[8];
    int map_width = map->info.width;
    getNeibors(neibors, position_num, map_width);
    for (int i = 0; i < 8; ++i) {
      if (neibors[i] >= 0 && neibors[i] < map->data.size()) {
        if (map->data[neibors[i]] == -1) {
          return true;
        }
      }
    }
  }
}

/**
 *  @brief  deduct 8 neibors of a point in a map
 *  @param  const pointer to msg type nav_msgs/OccupancyGrid
 *  @param  integer of a point in the map
 *  @return void
 */
void Frontier::getNeibors(int neibors[], int position_num,
                          int map_width) {
  /*
   * only get integer order number in the map, start from the upper left one.
   */
  neibors[0] = position_num - map_width - 1;
  neibors[1] = position_num - map_width;
  neibors[2] = position_num - map_width + 1;
  neibors[3] = position_num - 1;
  neibors[4] = position_num + 1;
  neibors[5] = position_num + map_width - 1;
  neibors[6] = position_num + map_width;
  neibors[7] = position_num + map_width + 1;
}

/**
 *  @brief  get info from /map and calculate the available frontiers' median
 *  points for the turtlebot.
 *  @param  integer of argument count
 *  @param  char pointer of argument value
 *  @return a bool value
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "frontierFinder");
  ros::NodeHandle n;
  Frontier frontierFinder;
  /**
   * invoke a call to the ROS master node. register publishing to
   * "/frontierPossibilities" and subscribing to "/map".
   * size of the message queue are both 1 for subscriber and publisher.
   */
  ros::Publisher frontierPossibilities_pub = n
      .advertise<sensor_msgs::PointCloud>("/frontierPossibilities", 1);
  ros::Subscriber frontierPossibilities_sub = n.subscribe(
      "/map", 1, &Frontier::frontierTarget,
                                &frontierFinder);
  Frontier::frontierGoal.header.frame_id = "map";
  ros::Rate loop_rate(30);
  while (ros::ok() && n.ok()) {
    frontierPossibilities_pub.publish(Frontier::frontierGoal);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
