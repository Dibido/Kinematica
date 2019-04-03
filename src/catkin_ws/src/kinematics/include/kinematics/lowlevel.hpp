/**
 * @file lowlevel.hpp
 * @author Dibran & Marnix
 * @brief lowlevel interface
 * @version 1.0
 * @date 2019-02-18
 * @copyright Copyright (c) 2019
 */

#ifndef LOWLEVEL_H_
#define LOWLEVEL_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/asio.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <stdexcept>
#include <algorithm>

#include "../src/Servo.h"

// Min and max move time in milliseconds
#define MIN_MOVE_TIME 100
#define MAX_MOVE_TIME 65535

// The pulsewidth range accepted by the hardware
#define MIN_PULSEWIDTH 500
#define MAX_PULSEWIDTH 2500

#define JOINT_0_PULSEWIDTH_COMPENSATION -50
#define JOINT_1_PULSEWIDTH_COMPENSATION 30
#define JOINT_2_PULSEWIDTH_COMPENSATION 350
#define JOINT_3_PULSEWIDTH_COMPENSATION 0
#define JOINT_4_PULSEWIDTH_COMPENSATION 0
#define JOINT_5_PULSEWIDTH_COMPENSATION 0

// The time it takes for a servo to move a single degree
// https://www.servocity.com/hs-805bb-servo
// No-Load Speed (4.8V) 	0.19sec/60° evaluates to 3.16 in 1ms
#define MS_PER_DEGREE 3.16

class lowlevel
{
public:
  /**
   * @brief Construct a new lowlevel object
   */
  lowlevel();

  /**
   * @brief Destroy the lowlevel object
   */
  virtual ~lowlevel();

  boost::asio::io_service ioservice;
  boost::asio::serial_port serial;

  /**
   * @brief Sends the command to move a servo to a certain angle in a certain amount if time.
   * @param aPin - The pin number of the servo
   * @param aDegrees - The angle in degrees
   * @param aMillis - The time in milliseconds that will be taken to complete the move
   * (if lower then MIN_MOVE_TIME, MIN_MOVE_TIME is used), same goes for MAX_MOVE_TIME.
   * @return - whether the move succeeded
   */
  bool moveServosToPos(std::vector<unsigned int> aPins, std::vector<int> aDegrees, unsigned int aMillis);

  /**
   * @brief Converts a given amount of degrees to a corresponding pulsewidth. Uses the MIN/MAX_PULSEWIDTH defines for this.
   * @param aDegrees - The angle in degrees 
   * @param aServo - The servo, is used to check boundaries (min/max range).
   */
  unsigned int convertDegreesToPulsewidth(int aDegrees, Servo& aServo) const;

  /**
   * @brief stop the movement of the servos
   * @param aPins  - The pins of the servos
   */
  void stopServos(std::vector<unsigned int> aPins);

  /**
   * @brief Checks whether a given amount of degrees is in range
   * @param aDegrees - The given amount of degrees
   * @param aServo - The servo object, which contains a min/max
   * @return - true if min <= aDegrees <= max
   */
  bool degreesInRange(int aDegrees, Servo& aServo) const;

  /**
   * @brief Sends a command over serial
   * @param aCommand - The command
   */
  void sendSerial(std::string aCommand);

  /**
   * @brief Set the baud rate of the serial
   * @param aBaudRate - The baudrate to set
   */
  void setBaudRate(unsigned int aBaudRate);

  /**
   * @brief Checks whether a servo with given servoId/pin exists
   * @param aPin - The given servoId
   * @return Returns true if there exists a servo in mServos with the given id, returns false otherwise
   */
  bool servoExists(unsigned int aServoId) const;

  /**
   * @brief Get the servo from mServos corresponding with given servo id
   * If the servo does not exist a invalidArgument exception is thrown, 
   * recommended to use method servoExists before getting it with this method.
   * @param aServoId - The given servo id
   * @return - A reference to the requested Servo
   */
  Servo& getServoWithId(unsigned int aServoId);

  /**
    * @brief Set the mArmLocked variable.
    * @param aLocked - True = locked, false = unlocked
    */
  void setArmLocked(bool aLocked);

  /**
    * @brief Whether the arm is locked or not
    * @return True if arm locked, false if not   
    */
  bool isArmLocked() const;

  /**
   * @brief Checks whether the given change in degrees for each servo can be reached within the target timeframe
   * If it is not possible a warning will be generated, and the actual time needed will be returned.
   * @param aPins - The pins of the servo's that will be moved
   * @param aDegrees - The target degrees for each servo
   * @param aMillis - The target timeframe in milliseconds
   * @return The minimum time needed to complete the move.
   */
  unsigned int checkTimeToMoveInRange(std::vector<unsigned int> aPins, std::vector<int> aDegrees, unsigned int aMillis);

private:
  /**
   * @brief If true, the low level driver refuses to send any move commands to the arm.
   * Is set true when for emergency stop for example.
   */
  bool mArmLocked;

  /**
   * @brief A list of servo objects
   */
  std::vector<Servo> mServos;

  /**
   * @brief Maps the value from the input range to the output range
   * @param aDegree - The value to convert
   * @param aInMin - The minimum value of the input range
   * @param aInMax - The maximum value of the input range
   * @param aOutMin - The minimum value of the output range
   * @param aOutMax - The maximum value of the output range 
   * @return unsigned int - The converted value
   */
  unsigned int mapValues(int aDegree, int aInMin, int aInMax, int aOutMin, int aOutMax) const;
};

#endif