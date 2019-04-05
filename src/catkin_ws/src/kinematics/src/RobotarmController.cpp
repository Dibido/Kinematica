#include "RobotarmController.h"

/* Base height is 6.1, we try to have effector 2 cm above ground so compensate around -4.1, 
      values have been tweaked slightly as it visibly improved the behavior of the robot */
const double RobotConstants::BEFORE_HEIGHT_COMPENSATION = 4.193;
const double RobotConstants::BEFORE_HEIGHT_BASE_COMPENSATION = 8.193;
const double RobotConstants::BASE_HEIGHT_COMPENSATION = -5.793;

// Gripper values
const unsigned int RobotConstants::GRIPPER_OPEN_DEGREES = 60;
const unsigned int RobotConstants::GRIPPER_CLOSED_DEGREES = 180;
const double RobotConstants::GRIPPER_OPEN_WIDTH_CM = 3.0;
const double RobotConstants::GRIPPER_CLOSED_WIDTH_CM = 0.0;
const unsigned int RobotConstants::GRIPPER_CLOSE_OFFSET = 40;

// Shape size detection compensation
const double RobotConstants::SHAPE_SIZE_COMPENSATION = 0.5;

// Relevant side lengths of the robot in CM (0 = shoulder to elbow, 1 = elbow to wrist, 2 = wrist to gripper)
Matrix<double, 3, 1> RobotConstants::cSidelengths = {{{14.605}},
                                                     {{18.733}},
                                                     {{10.0}}};

// Initial theta values
Matrix<double, 3, 1> RobotConstants::cStartThetas = {{{0}},
                                                     {{0}},
                                                     {{90.0}}};

// The allowed theta ranges, row 1 contains min max of theta 1 in degrees etc.
Matrix<double, 3, 2> RobotConstants::cThetaRanges = {{{-30}, {90}}, {{0}, {135}}, {{-90}, {90}}};

RobotarmController::RobotarmController(int aCamIndex) : mCurrentThetas(RobotConstants::cStartThetas), mShape(Shape()), mCamIndex(aCamIndex)
{
  ros::NodeHandle lNodeHandler;
  mMoveServosPublisher = lNodeHandler.advertise<kinematics::moveServos>("moveServos", 1000);

  initialize();
}

RobotarmController::~RobotarmController()
{
}

void RobotarmController::run()
{
  while (true)
  {
    while (!retrieveObject())
    {
      std::cout << "Unable to find correct shape, Looking for : " << mShapeSizeRequirements.at(0,0) << "x" << mShapeSizeRequirements.at(1,0) << " Actual shape is :" << mShape.mShapeWidth << "x" << mShape.mShapeHeight << "." << std::endl;
    }
    planAndExecuteRoute();
    std::cout << "Select a new shape" << std::endl;
  }
}

void RobotarmController::initialize()
{
  // Retrieve pixel per cm ratio
  mPixelsPerCm = mShapeDetector.calibrateCoordinates(mCamIndex);

  // Get the robotarm base coordinates
  mRobotBase = mShapeDetector.calibrateRobotarmBase(mPixelsPerCm, mCamIndex);

  // Locating drop point
  mDropPoint = mShapeDetector.detectBaseCoordinates(mCamIndex);

  // Converting point to cm
  mDropPoint = mDropPoint * (1.0 / mPixelsPerCm);
}

bool RobotarmController::retrieveObject()
{
  bool lReturn = false;
  std::pair<std::vector<Shape>, Matrix<double, 2, 1>> lShapeReturnValue;
  lShapeReturnValue = mShapeDetector.detectShapeCoordinates(mCamIndex);

  mShapeSizeRequirements = lShapeReturnValue.second;

  for(Shape lShape : lShapeReturnValue.first)
  {
    mShape = lShape;
    // Convert to cm
    mShape.mCenterPoint = mShape.mCenterPoint * (1.0 / mPixelsPerCm);
    mShape.mShapeWidth = mShape.mShapeWidth * (1.0 / mPixelsPerCm);
    mShape.mShapeHeight = mShape.mShapeHeight * (1.0 / mPixelsPerCm);
    // Check if found shape meets requirements
    if(mShape.mShapeWidth <= mShapeSizeRequirements.at(0,0) + (mShape.mShapeWidth * RobotConstants::SHAPE_SIZE_COMPENSATION) && mShape.mShapeWidth >= mShapeSizeRequirements.at(0,0) - (mShape.mShapeWidth * RobotConstants::SHAPE_SIZE_COMPENSATION)
      && mShape.mShapeHeight <= mShapeSizeRequirements.at(1,0) + (mShape.mShapeHeight * RobotConstants::SHAPE_SIZE_COMPENSATION) && mShape.mShapeHeight >= mShapeSizeRequirements.at(1,0) - (mShape.mShapeHeight * RobotConstants::SHAPE_SIZE_COMPENSATION))
    {
      return true;
    }
  }
  return lReturn;
}

bool RobotarmController::planAndExecuteRoute()
{
  // Calculate base angle to shape (when looking from above, considering horizontal line as X-axis of robot base)
  double lShapeAngle = MatrixFunctions::calculateBaseAngle(mRobotBase, mShape.mCenterPoint);

  std::cout << "lShapeAngle : " << lShapeAngle <<  std::endl;
  std::cout << "mShape.mShapeAngle : " << mShape.mShapeAngle << std::endl;

  // Calculate the angle of the gripper
  double lGripperAngle = lShapeAngle - mShape.mShapeAngle;

  if(lGripperAngle > 90.0)
  {
    lGripperAngle -= 180;
  }

  std::cout << "lGripperAngle : " << lGripperAngle << std::endl;

  // Calculate the distance to the object
  double lDistanceToObject = MatrixFunctions::calcDistance(mRobotBase, mShape.mCenterPoint);

  // Calculate base angle to drop point (when looking from above, considering horizontal line as X-axis of robot base)
  double lDropAngle = MatrixFunctions::calculateBaseAngle(mRobotBase, mDropPoint);

  // Calculate the distance to the drop point
  double lDistanceToDropPoint = MatrixFunctions::calcDistance(mRobotBase, mDropPoint);

  // Calculate how far we'll need to close the gripper to pick up the object using mapping.
  double lClosedGripperDegrees = mapValues(mShape.mShapeWidth, RobotConstants::GRIPPER_CLOSED_WIDTH_CM, RobotConstants::GRIPPER_OPEN_WIDTH_CM, RobotConstants::GRIPPER_CLOSED_DEGREES, RobotConstants::GRIPPER_OPEN_DEGREES);

  lClosedGripperDegrees += RobotConstants::GRIPPER_CLOSE_OFFSET;

  // double lApproachAngle = baseApproachAngleToShape();

  /* We have all the information needed, its time to plan the route. As we want to approach the object from above,
  we will first need to move to a point above the shape. Then we will move down to the object. Next we want to close
  our gripper, and move upwards. Then we want to move to a point above the drop point. Then we will lower the arm towards the drop point.
  At last we want to release our gripper so the object gets dropped at the drop point, to then move the arm upwards again */

  // All different configurations will be computed first, to check if a sequence is actually possible.
  std::vector<std::pair<bool, Matrix<double, 3, 1>>> lConfigurations;

  Matrix<double, 2, 1> lAboveShapeEffector = {{{lDistanceToObject}}, {{RobotConstants::BEFORE_HEIGHT_COMPENSATION}}};

  std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixFunctions::computeConfiguration(lAboveShapeEffector, RobotConstants::cSidelengths, mCurrentThetas, RobotConstants::cThetaRanges, 50);
  lConfigurations.push_back(lConfiguration);
  mCurrentThetas = lConfiguration.second;

  Matrix<double, 2, 1> lShapeEffector = {{{lDistanceToObject}}, {{RobotConstants::BASE_HEIGHT_COMPENSATION}}};
  lConfiguration = MatrixFunctions::computeConfiguration(lShapeEffector, RobotConstants::cSidelengths, mCurrentThetas, RobotConstants::cThetaRanges, 50);
  lConfigurations.push_back(lConfiguration);
  mCurrentThetas = lConfiguration.second;

  Matrix<double, 2, 1> lAboveDropPointEffector = {{{lDistanceToDropPoint}}, {{RobotConstants::BEFORE_HEIGHT_COMPENSATION}}};
  lConfiguration = MatrixFunctions::computeConfiguration(lAboveDropPointEffector, RobotConstants::cSidelengths, mCurrentThetas, RobotConstants::cThetaRanges, 50);
  lConfigurations.push_back(lConfiguration);
  mCurrentThetas = lConfiguration.second;

  Matrix<double, 2, 1> lDropPointEffector = {{{lDistanceToDropPoint}}, {{RobotConstants::BASE_HEIGHT_COMPENSATION}}};
  lConfiguration = MatrixFunctions::computeConfiguration(lDropPointEffector, RobotConstants::cSidelengths, mCurrentThetas, RobotConstants::cThetaRanges, 50);
  lConfigurations.push_back(lConfiguration);
  mCurrentThetas = lConfiguration.second;

  Matrix<double, 3, 1> lParkConfiguration;
  // Set the ready position servo values
  lParkConfiguration = {{{ -30 }, {{ 135 }}, {{ 60 }}}};

  // Check if there exist valid configurations for the 4 points.
  for (size_t lIndex = 0; lIndex < lConfigurations.size(); ++lIndex)
  {
    if (lConfigurations.at(lIndex).first == false)
    {
      std::cout << "Couldn't compute configuration for desired end effector" << std::endl;
      return false;
    }
  }

  /* Executing of route, each move will take 0.8 seconds to complete.
  Note how servo's with channel 1/3 have their theta inverted,
  the servo's are inverted (probably due to wrong mechanical assembly) */
  // Action (1/9), moving gripper to above the object.
  moveRobotarmToPosition(lShapeAngle, lConfigurations.at(0).second, lGripperAngle, RobotConstants::GRIPPER_OPEN_DEGREES, 800, 100.0);
  // Action (2/9), moving gripper down to the object.
  moveRobotarmToPosition(lShapeAngle, lConfigurations.at(1).second, lGripperAngle, RobotConstants::GRIPPER_OPEN_DEGREES, 800, 100.0);
  // Action (3/9), closing gripper.
  moveRobotarmToPosition(lShapeAngle, lConfigurations.at(1).second, lGripperAngle, lClosedGripperDegrees, 800, 100.0);
  // Action (4/9), moving upwards.
  moveRobotarmToPosition(lShapeAngle, lConfigurations.at(0).second, lGripperAngle, lClosedGripperDegrees, 800, 100.0);
  // Action (5/9), moving to above the dropping point.
  moveRobotarmToPosition(lDropAngle, lConfigurations.at(2).second, 0, lClosedGripperDegrees, 800, 100.0);
  // Action (6/9), moving down to the dropping point.
  moveRobotarmToPosition(lDropAngle, lConfigurations.at(3).second, 0, lClosedGripperDegrees, 800, 100.0);
  // Action (7/9), opening the gripper dropping the object.
  moveRobotarmToPosition(lDropAngle, lConfigurations.at(3).second, 0, RobotConstants::GRIPPER_OPEN_DEGREES, 800, 100.0);
  // Action (8/9), moving gripper upwards again.
  moveRobotarmToPosition(lDropAngle, lConfigurations.at(2).second, 0, RobotConstants::GRIPPER_OPEN_DEGREES, 800, 100.0);
  // Action (9/9), moving back to starting position.
  moveRobotarmToPosition(0, lParkConfiguration, 0, RobotConstants::GRIPPER_OPEN_DEGREES, 800, 100.0);
  return true;
}

double RobotarmController::mapValues(double aDegree, int aInMin, int aInMax, int aOutMin, int aOutMax)
{
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}

void RobotarmController::moveRobotarmToPosition(double aShapeAngle, Matrix<double, 3, 1> aConfiguration, double aGripperRotationDegree, double aGripperDegree, unsigned int aMoveTime, double aMoveTimeDelay)
{
  kinematics::moveServos lMoveServosMessage;
  kinematics::servoPosition lServoPosition;

  lMoveServosMessage.time = aMoveTime;

  lMoveServosMessage.servos.clear();
  lServoPosition.servoId = 0;
  lServoPosition.position = static_cast<int>(aShapeAngle);
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 1;
  lServoPosition.position = static_cast<int>(-aConfiguration.at(0,0));
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 2;
  lServoPosition.position = static_cast<int>(aConfiguration.at(1,0));
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 3;
  lServoPosition.position = static_cast<int>(-aConfiguration.at(2,0));
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 4;
  lServoPosition.position = static_cast<int>(aGripperDegree);
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 5;
  lServoPosition.position = static_cast<int>(aGripperRotationDegree);
  lMoveServosMessage.servos.push_back(lServoPosition);

  ROS_INFO("Sending move servos message");
  mMoveServosPublisher.publish(lMoveServosMessage);
  ros::spinOnce();

  // Sleep for the time that is needed to complete the move, plus a small margin to make sure the move is completed.
  sleep(static_cast<unsigned int>((lMoveServosMessage.time + aMoveTimeDelay) / 1000.0));
}