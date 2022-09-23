/**
 * @file Goals_Test.cpp
 * @brief This file contains all tests for Goals Class.
 */

/* --Includes-- */
#include "../include/Goals.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "../include/Cleaner.h"


/**
 * @brief      Tests whether the Goal Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_Test         Test Name
 */
TEST(TESTSuite, Goal_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Cleaner Bot(Goal);
Goals Clean;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
Clean.Set_Current_X(Point, goal);
EXPECT_EQ(Clean.Get_Goal , Point);
}

/**
 * @brief      Tests whether the Goal X Point is sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_X_Test         Test Name
 */
TEST(TESTSuite, Goal_X_Test) {
std::vector<std::vector<double>> Goal = {{0, 0, 0}};
ros::NodeHandle n;
Goals V;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(V.Set_Current_X(Point, goal) , 0);
}

/**
 * @brief      Tests whether the Goal Y Point is Sent correctly
 *
 * @param[in]  TESTSuite      gtest framework
 * @param[in]  Goal_Y_Test         Test Name
 */

TEST(TESTSuite, Goal_Y_Test) {
std::vector<std::vector<double>> Goal = {{0, 1, 0}};
ros::NodeHandle n;
Goals V;
move_base_msgs::MoveBaseGoal goal;
auto Point = Goal.at(0);
EXPECT_EQ(V.Set_Current_Y(Point, goal) , 1);
}
