/**
 * @file main.cpp
 * @brief A program that drives turtlebot vacuum in a known map by
 */

/* --Includes-- */
#include "../include/Goals.h"
#include "../include/Cleaner.h"

/**
 * @brief      main  Execution starts here
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     Returns 0 upon successful execution
 */
int main(int argc, char **argv) {
const std::vector<std::vector<double>> Goal_Points{{4.5, 0.1, 90},
  {4.6, 0.5, 180},
  {0, 0.5, 90}, {0, 0.75, 0}, {3.5, 0.8, 90}, {0.9, 1.2, 0}, {3.5, 2, 0},
  {0.9, 2, 90}, {0.9, 2.5, 0}, {4.6, 2.7, 90}, {4.6, 3, 180}, {0.9, 3, 90},
  {0.9, 3.5, 0}, {4.6, 3.5, 90}, {4.6, 4, 180}, {0, 3.8, 90}, {0, 4.5, 0},
  {4.6, 4.5, 90}, {0, 0, -90}};
  ros::init(argc, argv, "Clean");
  ros::NodeHandle n;
  Cleaner Bot(Goal_Points);
  Bot.Clean_Room();
  return 0;
}
