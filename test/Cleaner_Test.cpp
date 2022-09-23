/**
 * @file Cleaner_Test.cpp
 * @brief This file contains all tests for Cleaner Class.
 */

/* --Includes-- */
#include "../include/Cleaner.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "../include/Goals.h"


/**
 * @brief      Tests whether client comes online
 *             when map server is online.        
 *
 * @param[in]  TESTSuite                 gtest framework
 * @param[in]  Server_Existance_Test     Test Name
 */

TEST(TESTSuite, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                                          ac("move_base", true);
  // Check the Existence of Service
  bool exists(ac.waitForServer(ros::Duration(10)));
  EXPECT_FALSE(exists);
}

/**
 * @brief      Tests whether the Goal X Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  X_Test         Test Name
 */
TEST(TESTSuite, X_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(Clean.Set_Current_X(Point, goal) , 0);
}

/**
 * @brief      Tests whether the Goal Y Point is Sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Y_Test         Test Name
 */

TEST(TESTSuite, Y_Test) {
std::vector<std::vector<double>> Goal = {{0, 1, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(Clean.Set_Current_Y(Point, goal) , 1);
}
