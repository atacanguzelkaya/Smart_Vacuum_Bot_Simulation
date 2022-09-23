/**
 * @file Cleaner.h
 * @brief This file contains function declarations for Cleaner class. 
 */

/* --Includes-- */
#ifndef CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
#define CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
#include "Goals.h"
#include <vector>

/**
 * @brief Cleaner class
 *        This class is a protected derived class of base class Goals.
 *        This class contains functions which initialize action client.
 *        This class also uses functions from Goals class to send goals
 *        sequentially to move_base.
 */
class Cleaner:protected Goals {
 public:  // Public access specifier
/**
 * @brief Constructor for Cleaner Class
 *        Takes Goal Points in _Goals and stores them in a vetor of vectors
 *        named Goal_Points.
 * @param _Goals vector of vectors of type double
 */
explicit Cleaner(const std::vector<std::vector<double>>& _Goals);
/**
 * @brief Clean_Room Initializes action client to communicate with move_base
 *                   This function sends goals to move_base.
 */
void Clean_Room();
/**
 * @brief Destructor for Cleaner Class
 */
virtual ~Cleaner();
 private:
/** Create move_base message to send goal*/
move_base_msgs::MoveBaseGoal goal;
};
#endif  // CATKIN_WS_SRC_VACUUM_BOT_INCLUDE_CLEANER_H_
