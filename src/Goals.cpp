/**
 * @file Goals.cpp
 * @brief This file contains function definitions for Goals class.
 */

/* --Includes-- */
#include <vector>
#include "../include/Goals.h"

Goals::Goals() {
}
double Goals::Set_Current_X(std::vector<double>& Next_Goal,
                          move_base_msgs::MoveBaseGoal & goal_point) {
goal_point.target_pose.pose.position.x = Next_Goal.at(0);  // Goal X Coordinate
ROS_DEBUG("Goal X Point Successfully Set");
Get_Goal = Next_Goal;
return Next_Goal.at(0);  // Return goal point's X coordinate.
}

double Goals::Set_Current_Y(std::vector<double>& Next_Goal,
                          move_base_msgs::MoveBaseGoal & goal_point) {
goal_point.target_pose.pose.position.y = Next_Goal.at(1);  // Goal Y Coordinate
ROS_DEBUG("Goal Y Point Successfully Set");
return Next_Goal.at(1);  // Return goal point's Y coordinate.
}

void Goals::Set_Current_Orientation(std::vector<double>& Next_Goal,
                                   move_base_msgs::MoveBaseGoal & goal_point) {
auto Angle_Degrees = Next_Goal.at(2);  // Extract Orientation Value
auto Radians = Angle_Degrees*(3.14159/180);  // Convert Degrees to Radians
auto quaternion = tf::createQuaternionFromYaw(Radians);  // Create Quaternion
geometry_msgs::Quaternion qMsg;
tf::quaternionTFToMsg(quaternion, qMsg);  // Quaternion to Quaternion msg
goal_point.target_pose.pose.orientation = qMsg;  // Set Goal Orientation
ROS_DEBUG("Goal Orientation Successfully Set");
}

Goals::~Goals() {
}
