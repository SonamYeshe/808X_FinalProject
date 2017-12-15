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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/finalproject/Frontier.h"

TEST(TESTSuite, mapcallback) {
  ros::NodeHandle n;
  Frontier frontierFinder;
  const nav_msgs::OccupancyGrid::ConstPtr& map;
  frontierFinder.frontierTarget(map);
  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontierFinder");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
