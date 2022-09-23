/**
 * @file Goals.h
 * @brief This file contains function declarations for Goals class. 
 */

/* --Includes-- */
#ifndef CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_GOALS_H_
#define CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_GOALS_H_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <vector>

/**
 * @brief Goals class
 *        This class is a base class which contains general use functions.
 *        This class contains functions which sets goal point's X,Y,orientation.
 */
class Goals{
 public:  // Public Access Specifier
/**
 * @brief Constructor for Goals Class
 */
Goals();

/**
 * @brief Set_Current_X a function to set goal's X coordinate.
 *        This function sets the goal's X coordinate when given a goal point.
 * @param Next_Goal is a Goal point of type double vector.
 * @param goal_point is a message of type move_base_msgs::MoveBaseGoal
 * @return double type Goal Point's X coordinate.
 */
double Set_Current_X(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal_point);

/**
 * @brief Set_Current_Y a function to set goal's Y coordinate.
 *        This function sets the goal's Y coordinate when given a goal point.
 * @param Next_Goal is a Goal point of type double vector.
 * @param goal_point is a message of type move_base_msgs::MoveBaseGoal
 * @return double type Goal Point's Y coordinate.
 */
double Set_Current_Y(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal_point);

/** Get_Goal of type double vector, Stores Current Goal Point Data */
std::vector<double> Get_Goal;

/**
 * @brief Destructor for Goals Class
 */
virtual ~Goals();

 protected:  // Protected Access Specifier
/** Goal_Points of type vector of double vectors, Stores all Goal Points */
std::vector<std::vector<double>> Goal_Points;

/**
 * @brief Set_Current_Orientation a function to set goal's Orientation.
 *        This function sets the goal's orientation when given a goal point.
 * @param Next_Goal is a Goal point of type double vector.
 * @param goal_point is a message of type move_base_msgs::MoveBaseGoal
 */
void Set_Current_Orientation(std::vector<double>& Next_Goal,
                        move_base_msgs::MoveBaseGoal & goal_point);

 private:
/** Create move_base message to send goal*/
move_base_msgs::MoveBaseGoal goal_point;
};
#endif  // CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_GOALS_H_
