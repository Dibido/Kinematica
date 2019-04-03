#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include <ros/ros.h>

// Message from other package
#include "robotarminterface/moveServos.h"

#include "Shapedetector.h"
#include "Matrix.hpp"
#include "MatrixFunctions.h"
#include "Shape.h"

// Robot constants, initialized in cpp
namespace RobotConstants
{
/* Base height is 6.1, we try to have effector 2 cm above ground so compensate around -4.1, 
values have been tweaked slightly as it visibly improved the behavior of the robot */
extern const double BEFORE_HEIGHT_COMPENSATION;
extern const double BEFORE_HEIGHT_BASE_COMPENSATION;
extern const double BASE_HEIGHT_COMPENSATION;

// Gripper values
extern const unsigned int GRIPPER_OPEN_DEGREES;
extern const unsigned int GRIPPER_CLOSED_DEGREES;
extern const double GRIPPER_OPEN_WIDTH_CM;
extern const double GRIPPER_CLOSED_WIDTH_CM;
extern const unsigned int GRIPPER_CLOSE_OFFSET;

// Relevant side lengths of the robot in CM (0 = shoulder to elbow, 1 = elbow to wrist, 2 = wrist to gripper)
extern Matrix<double, 3, 1> cSidelengths;

// Initial theta values
extern Matrix<double, 3, 1> cStartThetas;

// The allowed theta ranges, row 1 contains min max of theta 1 in degrees etc.
extern Matrix<double, 3, 2> cThetaRanges;

} // namespace RobotConstants

class RobotarmController
{
public:
  /**
   * @brief Construct a new Robotarm Controller object
   * @param aCamIndex - Index of the cam used
   */
  RobotarmController(int aCamIndex);
  
  ~RobotarmController();

  /**
   * @brief Main function which is blocking, all logic should happen within the run.
   */
  void run();

  /**
   * @brief Initializing function, is called at the end of the constructor.
   * Used to find things like coordinates of robot, pixels per cm etc.
   * @param aCamIndex - Index of the cam used
   */
  void initialize();

  /**
   * @brief This function will let you locate and set a shape
   */
  void retrieveObject();

  /**
   * @brief When an object is retrieved, this function will plan and execute the full route to
   * pick up the object. It will deliver the object to the drop point. Note that planning a route
   * may not be possible depending on location of the drop point and object.
   * @bool Returns true if route was succesfully executed, false if its not possible to plan a route.
   */
  bool planAndExecuteRoute();
  
  /**
   * @brief Converts a given value to the corresponding centimeters, uses mPixelsPerCm for this.
   * @param aValue - Value of pixels
   * @return Corresponding amount of centimeters.
   */
  double convertToCm(int aValue) const;
   
  Matrix<double, 3, 1> mCurrentThetas;

  Matrix<double, 3, 1> mGoalThetas;

  // Goal x,y when looking in a 2D plane, used for moving the effector away and up/down from the base
  Matrix<double, 2, 1> mGoalEffector;

  // Shape that robot will pick up.
  Shape mShape;

  // Base point of the robot base (x,y)
  Matrix<double, 2, 1> mRobotBase;

  // Point where to drop objects (in our situation location of the white circle)
  Matrix<double, 2, 1> mDropPoint;

  ros::Publisher mMoveServosPublisher;

  Shapedetector mShapeDetector;
  
  double mPixelsPerCm;

  int mCamIndex;
};

#endif /* ROBOTARMCONTROLLER_H_ */