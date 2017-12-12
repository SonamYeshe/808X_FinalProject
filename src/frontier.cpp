/*
 * frontier.cpp
 *
 *  Created on: Dec 10, 2017
 *      Author: sonamyeshe
 */
#include <vector>
#include <map>
#include <math.h>
#include "ros/ros.h"
#include "../include/finalproject/Frontier.h"
#include "nav_msgs/OccupancyGrid.h"

const int map_width = nav_msgs::MapMetaData::width;
// const int map_height = nav_msgs::MapMetaData::height;

int Frontier::frontierTarget(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::vector<int> frontierEdgeCell;
  for (int i = 0; i < map->data.size(); ++i) {
    if (isFrontierEdgeCell(map, i)) {
      frontierEdgeCell.push_back(i);
    }
  }
  std::vector<int> frontierEdgeOpen;
  std::map<int, std::vector<int> > frontierEdgeClose;

  for (int i = 0; frontierEdgeCell.size() > 0; ++i) {
    frontierEdgeOpen.push_back(frontierEdgeCell.back());
    while (!frontierEdgeOpen.empty()) {
      int neibors[8];
      int frontierCellUsed = frontierEdgeOpen.front();
      getNeibors(neibors, frontierCellUsed);
      for (int a = 0; a < 8; ++a) {
        for (int j = frontierEdgeCell.size() - 1; j >= 0; --j) {
          if (neibors[a] == frontierEdgeCell[j]) {
            for (int k = 0; k < frontierEdgeOpen.size(); ++k) {
              if (neibors[a] != frontierEdgeOpen[k]) {
                frontierEdgeOpen.push_back(neibors[a]);
              }
            }
          }
        }
      }
      frontierEdgeClose[i].push_back(frontierCellUsed);
      frontierEdgeOpen.erase(frontierEdgeOpen.begin());
    }
    for (int j = 0; j < frontierEdgeClose[i].size(); ++j) {
      for (int k = 0; k < frontierEdgeCell.size(); ++k) {
        if (frontierEdgeClose[i][j] == frontierEdgeCell[k]) {
          frontierEdgeCell.erase(frontierEdgeCell.begin() + k);
        }
      }
    }
  }
//  std::vector<int> frontierEdgeAvailableNum;
  std::vector<int> median;
  for (int i = 0; i < frontierEdgeClose.size(); ++i) {
    std::map<int, int> x;
    std::map<int, int> y;
    x[0] = frontierEdgeClose[i].front() / map_width;
    x[1] = frontierEdgeClose[i].back() / map_width;
    y[0] = frontierEdgeClose[i].front() % map_width;
    y[1] = frontierEdgeClose[i].back() % map_width;
    double edgeLength = sqrt(
        (double(x[1] - x[0])) ^ 2 + (double(y[1] - y[0])) ^ 2)
        * map->info.resolution;
    if (edgeLength > 0.5) {
//      frontierEdgeAvailableNum.push_back(i);
      median.push_back(frontierEdgeClose[i][frontierEdgeClose[i].size() / 2]);
    }
  }
  int turtlebot_position = round(
      -map->info.origin.position.y / map->info.resolution) * map_width
      + round(map->info.origin.position.x / map->info.resolution);
  double shortestLength = 1000000000000000;
  int finalTarget;
  for (int i = 0; i < median.size(); ++i) {
    std::map<int, int> x;
    std::map<int, int> y;
    x[0] = median[i] / map_width;
    x[1] = turtlebot_position / map_width;
    y[0] = median[i] % map_width;
    y[1] = turtlebot_position % map_width;
    double length = sqrt((double(x[1] - x[0])) ^ 2 + (double(y[1] - y[0])) ^ 2);
    if (shortestLength > length) {
      shortestLength = length;
      finalTarget = i;
    }
  }
  return median[finalTarget];
}

bool Frontier::isFrontierEdgeCell(const nav_msgs::OccupancyGrid::ConstPtr& map,
                                  int position_num) {
  if (map->data[position_num] != 0) {
    return false;
  } else {
    int neibors[8];
    getNeibors(neibors, position_num);
    for (int i = 0; i < 8; ++i) {
      if (neibors[i] == -1) {
        return true;
      }
    }
  }
}

void Frontier::getNeibors(int neibors[], int position_num) {
  neibors[0] = position_num - map_width - 1;
  neibors[1] = position_num - map_width;
  neibors[2] = position_num - map_width + 1;
  neibors[3] = position_num - 1;
  neibors[4] = position_num + 1;
  neibors[5] = position_num + map_width - 1;
  neibors[6] = position_num + map_width;
  neibors[7] = position_num + map_width + 1;
}
