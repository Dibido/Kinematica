/**
 * @file highlevel.h
 * @author Dibran & Marnix
 * @brief the high level driver for the robotarm
 * @version 1.0
 * @date 2019-02-18
 * @copyright Copyright (c) 2019
 */

#ifndef HIGHLEVEL_H_
#define HIGHLEVEL_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <signal.h>

#include <iostream>
#include <sstream>
#include <chrono>

#include "kinematics/lowlevel.hpp"
#include "kinematics/singleServo.h"
#include "kinematics/stopSingleServo.h"
#include "kinematics/moveServos.h"
#include "kinematics/stopServos.h"
#include "kinematics/armInstruction.h"

#define DEFAULT_BAUDRATE 115200

// Time robot will take to move to park position on initialization/shutdown
#define INITIALIZE_TIME 3000

/**
 * @brief Robotarmposition stores a configuration of the robotarm
 * @param servoIds - The id's of the servo's
 * @param servoDegrees - The degrees that the servo's are in
 * @param time - Defines period (in ms) the robot should take to move to this position
 */
struct robotarmPosition
{
  std::vector<unsigned int> servoIds;
  std::vector<int> servoDegrees;
  uint32_t time;

  /**
   * @brief Construct a new robotarm Position object
   */
  robotarmPosition() : time(0)
  {
  }

  /**
   * @brief Copy constructor
   * @param aRobotarmPosition - The object to be copied 
   */
  robotarmPosition(const robotarmPosition& aRobotarmPosition) : servoIds(aRobotarmPosition.servoIds), servoDegrees(aRobotarmPosition.servoDegrees), time(aRobotarmPosition.time)
  {
  }
};

class highlevel
{
public:
  /**
   * @brief Default constructor
   */
  highlevel();

  /**
   * @brief Constructor
   * @param aBaudRate 
   */
  highlevel(unsigned int aBaudRate);

  /**
   * @brief Destroy the highlevel object
   */
  virtual ~highlevel();

  /**
   * @brief Set the Baud Rate
   * @param aBaudRate - The baudrate to set
   */
  void setBaudRate(unsigned int aBaudRate);

  /**
   * @brief Base function used to keep the highlevel active, is a blocking function all logic should be handled within it.
   */
  void run();

private:
  ros::NodeHandle mNodeHandler;
  lowlevel mLowLevelDriver;
  ros::Subscriber mSingleServoSubscriber;
  ros::Subscriber mStopSingleServoSubscriber;
  ros::Subscriber mMoveServosSubscriber;
  ros::Subscriber mStopServosSubscriber;
  ros::Subscriber mArmInstructionSubscriber;

  robotarmPosition mParkPosition;
  robotarmPosition mReadyPosition;
  robotarmPosition mStraightPosition;

  /**
   * @brief False intitially, becomes true as soon as atleast one valid move command has been sent to the robot by serial
   */
  bool mMoveCommandSent;

  /**
   * @brief False initially, whether the system is in the ready state
   */
  bool mIsReady;

  /**
   * @brief False initially, should become true when the robot completed its first move to park position
   */
  bool mIsInitialized;

  /**
   * @brief List of all servo ids
   */
  std::vector<unsigned int> mServoIds;
  
  /**
   * @brief Any move commands received by message get stored in this queue of commands
   */
  std::vector<robotarmPosition> mMoveCommands;

  /**
   * @brief Time to take to go to the park position
   */
  unsigned int mInitializeTime;

  /**
   * @brief The currently set baudrate
   */
  unsigned int mBaudRate;

  /**
   * @brief Expected period of time to complete the last sent valid move. Is 0 until the first valid move is sent to the lowlevel.
   */
  uint32_t mLastMoveTimeMS;

  /**
   * @brief Start time of last sent valid move command.
   */
  std::chrono::high_resolution_clock::time_point mLastMoveStartTime;

  /**
   * @brief Is used to catch ctrl-c event to kill the application
   */
  static void signalHandler(int aSignal);

  // Message callbacks
  void singleServoCallback(const kinematics::singleServoConstPtr& aSingleServoMessage);
  void stopSingleServoCallback(const kinematics::stopSingleServoConstPtr& aStopSingleServoMessage);
  void moveServosCallback(const kinematics::moveServosConstPtr& aSingleServoMessage);
  void stopServosCallback(const kinematics::stopServosConstPtr& aStopAllServoMessage);
  void armInstructionCallback(const kinematics::armInstructionConstPtr& aArmPosition);

  /**
   * @brief Initializes the arm by going to the park position
   */
  void initializeArm();

  /**
   * @brief subscribes to the necessary topics
   */
  void subscribeTopics();

  /**
   * @brief Initializes the position and timing values
   */
  void initializeValues();

  /**
   * @brief Checks if new commands from the mMoveCommands should be sent to the low level driver
   */
  void handleMoveCommands();

  /**
   * @brief Checks if the period needed to complete last sent valid move has expired
   * @return whether the move has expired
   */
  bool lastMovePeriodExpired() const;
};

#endif