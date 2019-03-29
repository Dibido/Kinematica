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
#define BASE_HEIGHT_COMPENSATION -4.193

Matrix<double, 3, 1> gSidelengths = {{{14.605}},
                                     {{18.733}},
                                     {{10.0}}};

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
    gGoal = {{{0}}, {{BASE_HEIGHT_COMPENSATION}}};

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
      Matrix<double, 2, 1> lDetectedRobotarmBaseCoordinates;
      Matrix<double, 2, 1> lDetectedShapeCoordinates;
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
        lDetectedShapeCoordinates /= lCoordinateSystemConversionValue;
        lDetectedBaseCoordinates /= lCoordinateSystemConversionValue;
        // Modify the X/Y coordinates to be in the refence frame of the arm base
        std::cout << "Detected Shape: " << lDetectedShapeCoordinates << std::endl;
        std::cout << "Converted robotarm base coordinates : " << lDetectedRobotarmBaseCoordinates << std::endl;
        std::cout << "Detected Base: " << lDetectedBaseCoordinates << std::endl;

        // Calculate base angle
        double lBaseAngle = MatrixFunctions::calculateBaseAngle(lDetectedRobotarmBaseCoordinates, lDetectedShapeCoordinates);

        double lDeltaToObject = MatrixFunctions::calcDistance(lDetectedRobotarmBaseCoordinates, lDetectedShapeCoordinates);

        gGoal = {{{lDeltaToObject}}, {{BASE_HEIGHT_COMPENSATION}}};

        std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixFunctions::computeConfiguration(gGoal, gSidelengths, gThetas, gThetaRanges, 50);

        Matrix<double, 3, 1> lThetas = lConfiguration.second;

        if(lConfiguration.first == false)
        {
          throw std::logic_error("Couldn't compute configuration for desired end effector");
        }
        
        std::cout << "Arm angle : " << lBaseAngle << std::endl;
        // Send angle to higlevel
        robotarminterface::moveServos lMoveServosMessage;
        robotarminterface::servoPosition lServoPosition;
        lServoPosition.servoId = 0;
        lServoPosition.position = lBaseAngle;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 1;
        lServoPosition.position = lThetas[0][1];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 2;
        lServoPosition.position = lThetas[1][1];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 3;
        lServoPosition.position = -lThetas[2][1];
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 4;
        lServoPosition.position = 180;
        lMoveServosMessage.servos.push_back(lServoPosition);

        lServoPosition.servoId = 5;
        lServoPosition.position = 180;
        lMoveServosMessage.servos.push_back(lServoPosition);

        // Move time of 100ms is unrealistic, highlevel should show warning.
        lMoveServosMessage.time = 3000;
        ROS_INFO("Sending allServoPos");

        // Send message
        lMoveServosPublisher.publish(lMoveServosMessage);
        ros::spinOnce();
      }
      else
      {
        std::cout << "Invalid usage, expected form ./kinematics [webcamId]" << std::endl;
        exit(0);
      }

      std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixFunctions::computeConfiguration(gGoal, gSidelengths, gThetas, gThetaRanges, 10);

      if (lConfiguration.first == true)
      {
        std::cout << "Succes, configuration found for point(" << std::to_string(gGoal[0][0]) << "," << std::to_string(gGoal[1][0]) << "):" << std::endl;
        std::cout << lConfiguration.second << std::endl;
      }
      else
      {
        std::cout << "Failure, can't find configuration found for point(" << std::to_string(gGoal[0][0]) << "," << std::to_string(gGoal[1][0]) << "):" << std::endl;
      }
    }
  }
  catch (std::exception &e)
  {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
