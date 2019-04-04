#include "RobotarmController.h"

/* Base height is 6.1, we try to have effector 2 cm above ground so compensate around -4.1, 
      values have been tweaked slightly as it visibly improved the behavior of the robot */
const double RobotConstants::BEFORE_HEIGHT_COMPENSATION = 4.193;
const double RobotConstants::BEFORE_HEIGHT_BASE_COMPENSATION = 8.193;
const double RobotConstants::BASE_HEIGHT_COMPENSATION = -4.693;

// Gripper values
const unsigned int RobotConstants::GRIPPER_OPEN_DEGREES = 60;
const unsigned int RobotConstants::GRIPPER_CLOSED_DEGREES = 180;
const double RobotConstants::GRIPPER_OPEN_WIDTH_CM = 3.0;
const double RobotConstants::GRIPPER_CLOSED_WIDTH_CM = 0.0;
const unsigned int RobotConstants::GRIPPER_CLOSE_OFFSET = 20;

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

RobotarmController::RobotarmController(int aCamIndex) : mCurrentThetas(RobotConstants::cStartThetas), mCamIndex(aCamIndex)
{
  // Testing purposes:
  Matrix<double, 1, 2> aPoint1Start = {{{0}, {10}}};
  Matrix<double, 1, 2> aPoint1End = {{{50}, {10}}};
  Matrix<double, 1, 2> aPoint2Start = {{{0}, {20}}};
  Matrix<double, 1, 2> aPoint2End = {{{30}, {0}}};
  Matrix<double, 1, 2> aIntersectionPoint;

  bool lSucces = intersection(aPoint1Start, aPoint1End, aPoint2Start, aPoint2End, aIntersectionPoint);
 
  if(lSucces)
  {
    std::cout << "Succes, intersection found at: " << std::endl;
    std::cout << aIntersectionPoint << std::endl;
  }
  else
  {
    std::cout << "No intersection" << std::endl;
  }

  //
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
    retrieveObject();
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

void RobotarmController::retrieveObject()
{
  mShape = mShapeDetector.detectShapeCoordinates(mCamIndex);

  // mShape.mCenterPoint = lShape.first;
  // mShape.mShapeWidth = lShape.second;

  // Convert to cm

  mShape.mCenterPoint = mShape.mCenterPoint * (1.0 / mPixelsPerCm);
  mShape.mShapeWidth = mShape.mShapeWidth * (1.0 / mPixelsPerCm);
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

  double lApproachAngle = baseApproachAngleToShape();

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

  // Check if there exist valid configurations for the 4 points.
  for (int lIndex; lIndex < lConfigurations.size(); ++lIndex)
  {
    if (lConfigurations.at(lIndex).first == false)
    {
      std::cout << "Couldn't compute configuration for desired end effector" << std::endl;
      return false;
    }
  }

  /* Executing of route, each move will take 3 seconds to complete.
  Note how servo's with channel 1/3 have their theta inverted,
  the servo's are inverted (probably due to wrong mechanical assembly) */
  // Action (1/8), moving gripper to above the object.
  moveRobotarmToPosition(lShapeAngle, lConfigurations, 0, lGripperAngle, RobotConstants::GRIPPER_OPEN_DEGREES, 3000, 100.0);
  // Action (2/8), moving gripper down to the object.
  moveRobotarmToPosition(lShapeAngle, lConfigurations, 1, lGripperAngle, RobotConstants::GRIPPER_OPEN_DEGREES, 3000, 100.0);
  // Action (3/8), closing gripper.
  moveRobotarmToPosition(lShapeAngle, lConfigurations, 1, lGripperAngle, lClosedGripperDegrees, 3000, 100.0);
  // Action (4/8), moving upwards.
  moveRobotarmToPosition(lShapeAngle, lConfigurations, 0, lGripperAngle, lClosedGripperDegrees, 3000, 100.0);
  // Action (5/8), moving to above the dropping point.
  moveRobotarmToPosition(lDropAngle, lConfigurations, 2, 0, lClosedGripperDegrees, 3000, 100.0);
  // Action (6/8), moving down to the dropping point.
  moveRobotarmToPosition(lDropAngle, lConfigurations, 3, 0, lClosedGripperDegrees, 3000, 100.0);
  // Action (7/8), opening the gripper dropping the object.
  moveRobotarmToPosition(lDropAngle, lConfigurations, 3, 0, RobotConstants::GRIPPER_OPEN_DEGREES, 3000, 100.0);
  // Action (8/8), moving gripper upwards again.
  moveRobotarmToPosition(lDropAngle, lConfigurations, 2, 0, RobotConstants::GRIPPER_OPEN_DEGREES, 3000, 100.0);
  return true;
}

double RobotarmController::convertToCm(int aValue) const
{
  return static_cast<double>(aValue) / mPixelsPerCm;
}

double RobotarmController::mapValues(double aDegree, int aInMin, int aInMax, int aOutMin, int aOutMax) const
{
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}

void RobotarmController::moveRobotarmToPosition(double aShapeAngle, std::vector<std::pair<bool, Matrix<double, 3, 1>>> aConfiguration, unsigned int aConfigurationIndex, double aGripperRotationDegree, double aGripperDegree, unsigned int aMoveTime, double aMoveTimeDelay)
{
  kinematics::moveServos lMoveServosMessage;
  kinematics::servoPosition lServoPosition;

  lMoveServosMessage.time = aMoveTime;

  lMoveServosMessage.servos.clear();
  lServoPosition.servoId = 0;
  lServoPosition.position = aShapeAngle;
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 1;
  lServoPosition.position = -aConfiguration.at(aConfigurationIndex).second[0][0];
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 2;
  lServoPosition.position = aConfiguration.at(aConfigurationIndex).second[1][0];
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 3;
  lServoPosition.position = -aConfiguration.at(aConfigurationIndex).second[2][0];
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 4;
  lServoPosition.position = aGripperDegree;
  lMoveServosMessage.servos.push_back(lServoPosition);

  lServoPosition.servoId = 5;
  lServoPosition.position = aGripperRotationDegree;
  lMoveServosMessage.servos.push_back(lServoPosition);

  ROS_INFO("Sending move servos message");
  mMoveServosPublisher.publish(lMoveServosMessage);
  ros::spinOnce();

  // Sleep for the time that is needed to complete the move, plus a small margin to make sure the move is completed.
  sleep((lMoveServosMessage.time + aMoveTimeDelay) / 1000.0);
}