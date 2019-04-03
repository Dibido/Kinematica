// This can be done in any of the test files but then we will have multiple executables
// in one Eclipse project which is impossible in a non-makefile project

// See http://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/adv_scenarios/shared_lib_customizations/entry_point.html
#define BOOST_TEST_MODULE MatrixTestModule
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_NO_MAIN
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>

/// Local
#include "Shapedetector.h"
#include "Matrix.hpp"
#include "MatrixFunctions.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <thread>
#include <chrono>

#include <ros/ros.h>

#include "robotarminterface/singleServo.h"
#include "robotarminterface/stopSingleServo.h"
#include "robotarminterface/moveServos.h"
#include "robotarminterface/stopServos.h"
#include "robotarminterface/armInstruction.h"

#define UNIT_TEST false

// Base height is 6.1, we try to have effector 2 cm above ground so compensate -4.1
#define BEFORE_HEIGHT_COMPENSATION 4.193
#define BEFORE_HEIGHT_BASE_COMPENSATION 8.193
#define BASE_HEIGHT_COMPENSATION -4.693 // -2.193

// Gripper values
#define GRIPPER_OPEN_DEGREES 60
#define GRIPPER_CLOSED_DEGREES 180
#define GRIPPER_OPEN_WIDTH_CM 3.0
#define GRIPPER_CLOSED_WIDTH_CM 0.0
#define GRIPPER_CLOSE_OFFSET 10


Matrix<double, 3, 1> gSidelengths = {{{14.605}},
                                     {{18.733}}, /* 18.773 */
                                     {{10.0}}};

// Matrix<double, 3, 1> gSidelengths = {{{16.0}},
//                                     {{16.0}}, /* 18.773 */
//                                     {{12.0}}};

Matrix<double, 3, 1> gThetas = {{{0}},
                                {{0}},
                                {{90}}};

// The allowed theta ranges, row 1 contains min max of theta 1 in degrees etc.
Matrix<double, 3, 2> gThetaRanges = {{{-30}, {90}}, {{0}, {135}}, {{-90}, {90}}};

Matrix<double, 3, 1> gGoalConfiguration = {{{-25}},
                                           {{125}},
                                           {{80}}};

// Goal effector based on a valid gGoalConfiguration: 
Matrix<double, 2, 1>
    gShapeGoal = {{{0}}, {{BASE_HEIGHT_COMPENSATION}}};

Matrix<double, 2, 1>
gInbetweenShapeGoal = {{{0}}, {{BASE_HEIGHT_COMPENSATION}}};

Matrix<double, 2, 1>
    gBaseGoal = {{{0}}, {{BASE_HEIGHT_COMPENSATION}}};

Matrix<double, 2, 1>
gInbetweenBaseGoal = {{{0}}, {{BASE_HEIGHT_COMPENSATION}}};

Matrix<double, 2, 1> gDeltaEffector = {{{0}}, {{0}}};

double gDeltaFactor = 0.1;

int main(int argc,
         char **argv)
{
  try
  {
    ros::init(argc, argv, "servo_client");

    ros::NodeHandle lNodeHandler;
    ros::Publisher lSingleServoPublisher = lNodeHandler.advertise<robotarminterface::singleServo>("singleServo", 1000);
    ros::Publisher lStopSingleServoPublisher = lNodeHandler.advertise<robotarminterface::stopSingleServo>("stopSingleServo", 1000);
    ros::Publisher lMoveServosPublisher = lNodeHandler.advertise<robotarminterface::moveServos>("moveServos", 1000);
    ros::Publisher lStopServosPublisher = lNodeHandler.advertise<robotarminterface::stopServos>("stopServos", 1000);
    ros::Publisher lArmInstructionPublisher = lNodeHandler.advertise<robotarminterface::armInstruction>("armInstructionPosition", 1000);

    // In a real program we should test whether we should call the real main or run the unit_test_main
    if (UNIT_TEST)
    {
      return boost::unit_test::unit_test_main(&init_unit_test, argc, argv); // @suppress("Symbol is not resolved") // @suppress("Invalid arguments")
    }
    else
    {
      double lCoordinateSystemConversionValue;
      double lCloseGripperDegrees;
      Matrix<double, 2, 1> lDetectedRobotarmBaseCoordinates;
      std::pair<Matrix<double, 2, 1>, double> lDetectedShapeCoordinates;
      Matrix<double, 2, 1> lDetectedBaseCoordinates;
      if (argc > 1)
      {
        Shapedetector shapeDetector;
        // Get the pixel/centimeter size using calibration
        lCoordinateSystemConversionValue = shapeDetector.calibrateCoordinates(atoi(argv[1]));
        std::cout << "Calibration value : " << lCoordinateSystemConversionValue << "Pixels/CM" << std::endl;
        // Get the robotarm base coordinates
        lDetectedRobotarmBaseCoordinates = shapeDetector.calibrateRobotarmBase(lCoordinateSystemConversionValue, atoi(argv[1]));
        // Get target coordinates
        lDetectedShapeCoordinates = shapeDetector.detectShapeCoordinates(atoi(argv[1]));
        lDetectedBaseCoordinates = shapeDetector.detectBaseCoordinates(atoi(argv[1]));
        // Calculate new X/Y using calibration
        lDetectedShapeCoordinates.first /= lCoordinateSystemConversionValue;
        lDetectedShapeCoordinates.second /= lCoordinateSystemConversionValue;
        std::cout << "Detected shape width : " << lDetectedShapeCoordinates.second << std::endl;
        // Calculate closed gripper degrees
        lCloseGripperDegrees = ((lDetectedShapeCoordinates.second - GRIPPER_CLOSED_WIDTH_CM) * (GRIPPER_OPEN_DEGREES - GRIPPER_CLOSED_DEGREES) / (GRIPPER_OPEN_WIDTH_CM - GRIPPER_CLOSED_WIDTH_CM)) + GRIPPER_CLOSED_DEGREES;
        lCloseGripperDegrees += GRIPPER_CLOSE_OFFSET;
        std::cout << "lCloseGripperDegrees : " << lCloseGripperDegrees << std::endl;
        lDetectedBaseCoordinates /= lCoordinateSystemConversionValue;
        // Modify the X/Y coordinates to be in the refence frame of the arm base
        std::cout << "Detected Shape: " << lDetectedShapeCoordinates.first << std::endl;
        std::cout << "Converted robotarm base coordinates : " << lDetectedRobotarmBaseCoordinates << std::endl;

        /*******
         * Grab shape
         *******/
        // Calculate base angle for shape angle
        double lShapeAngle = MatrixFunctions::calculateBaseAngle(lDetectedRobotarmBaseCoordinates, lDetectedShapeCoordinates.first);
        // Calculate the distance to the object
        double lDeltaToObject = MatrixFunctions::calcDistance(lDetectedRobotarmBaseCoordinates, lDetectedShapeCoordinates.first);
        std::cout << "lDelta " << lDeltaToObject << std::endl;
        
        gShapeGoal = {{{lDeltaToObject}}, {{BASE_HEIGHT_COMPENSATION}}};
        gInbetweenShapeGoal = {{{lDeltaToObject}}, {{BEFORE_HEIGHT_COMPENSATION}}};

        std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixFunctions::computeConfiguration(gShapeGoal, gSidelengths, gThetas, gThetaRanges, 50);
        std::cout << "^In between goal" << std::endl;
        std::pair<bool, Matrix<double, 3, 1>> lBeforeConfiguration = MatrixFunctions::computeConfiguration(gInbetweenShapeGoal, gSidelengths, gThetas, gThetaRanges, 50);

        Matrix<double, 3, 1> lThetas = lConfiguration.second;
        Matrix<double, 3, 1> lBeforeThetas = lBeforeConfiguration.second;
        
        if(lConfiguration.first == false || lBeforeConfiguration.first == false)
        {
          throw std::logic_error("Couldn't compute configuration for desired end effector");
        }
        
        std::cout << "Arm angle : " << lShapeAngle << std::endl;
        // Move to above the shape
        robotarminterface::moveServos lMoveServosMessage;
        robotarminterface::servoPosition lServoPosition;
        lServoPosition.servoId = 0;
        lServoPosition.position = lShapeAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBeforeThetas[0][0]; /* + 2 */
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBeforeThetas[1][0]; /* + 30 */
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBeforeThetas[2][0]; /* - 5 */
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = 60;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        std::cout << "calculated Thetas : " << lThetas << std::endl;
        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");

        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(5);

        // Move down to shape
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lShapeAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = 60;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        std::cout << "calculated Thetas : " << lThetas << std::endl;

        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();

        // Close gripper
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lShapeAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = lCloseGripperDegrees;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        std::cout << "calculated Thetas : " << lThetas << std::endl;

        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();

        // Move to above the shape
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lShapeAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBeforeThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBeforeThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBeforeThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = lCloseGripperDegrees;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        std::cout << "calculated Thetas : " << lThetas << std::endl;
        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");

        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(5);
        
        /*******
         * Send shape to base
         * ******/
        // Calculate base angle for base angle
        double lBaseAngle = MatrixFunctions::calculateBaseAngle(lDetectedRobotarmBaseCoordinates, lDetectedBaseCoordinates);
        double lDeltaToBase = MatrixFunctions::calcDistance(lDetectedRobotarmBaseCoordinates, lDetectedBaseCoordinates);

        gBaseGoal = {{{lDeltaToBase}}, {{BASE_HEIGHT_COMPENSATION}}};
        gInbetweenBaseGoal = {{{lDeltaToBase}}, {{BEFORE_HEIGHT_BASE_COMPENSATION}}};

        std::pair<bool, Matrix<double, 3, 1>> lBaseConfiguration = MatrixFunctions::computeConfiguration(gBaseGoal, gSidelengths, gThetas, gThetaRanges, 50);
        std::pair<bool, Matrix<double, 3, 1>> lBeforeBaseConfiguration = MatrixFunctions::computeConfiguration(gInbetweenBaseGoal, gSidelengths, gThetas, gThetaRanges, 50);

        Matrix<double, 3, 1> lBaseThetas = lBaseConfiguration.second;
        Matrix<double, 3, 1> lBeforeBaseThetas = lBeforeBaseConfiguration.second;
        
        if(lBaseConfiguration.first == false || lBeforeBaseConfiguration.first == false)
        {
          throw std::logic_error("Couldn't compute configuration for desired end effector");
        }
        // Move to above base
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lBaseAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBeforeBaseThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBeforeBaseThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBeforeBaseThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = lCloseGripperDegrees;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(3);

        // Move down to base
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lBaseAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBaseThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBaseThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBaseThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = lCloseGripperDegrees;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(3);

        // Open gripper
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lBaseAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBaseThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBaseThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBaseThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = 60;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(3);

        // Move to above base
        lMoveServosMessage.servos.clear();
        lServoPosition.servoId = 0;
        lServoPosition.position = lBaseAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = -lBeforeBaseThetas[0][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lBeforeBaseThetas[1][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lBeforeBaseThetas[2][0];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = 60;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 10;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lMoveServosMessage.time = 5000;
        ROS_INFO("Sending allServoPos");
        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
        sleep(5);

        // Return to ready
        robotarminterface::armInstruction lArmInstructionMessage;
        lArmInstructionMessage.instruction = "park";
        lArmInstructionMessage.time = 2000;
        ROS_INFO("Sending armPosition park");
        //Send message
        lArmInstructionPublisher.publish(lArmInstructionMessage);
        ros::spinOnce();
        sleep(2);
      }
      else
      {
        std::cout << "Invalid usage, expected form ./kinematics [webcamId]" << std::endl;
        exit(0);
      }

      std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixFunctions::computeConfiguration(gShapeGoal, gSidelengths, gThetas, gThetaRanges, 10);

      if (lConfiguration.first == true)
      {
        std::cout << "Succes, configuration found for point(" << std::to_string(gShapeGoal[0][0]) << "," << std::to_string(gShapeGoal[1][0]) << "):" << std::endl;
        std::cout << lConfiguration.second << std::endl;
      }
      else
      {
        std::cout << "Failure, can't find configuration found for point(" << std::to_string(gShapeGoal[0][0]) << "," << std::to_string(gShapeGoal[1][0]) << "):" << std::endl;
      }
    }
  }
  catch (std::exception &e)
  {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
