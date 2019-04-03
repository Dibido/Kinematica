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
#include "RobotarmController.h"

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

int main(int argc,
         char **argv)
{
  if (argc >= 2)
  {
    RobotarmController lRobotarmController(atoi(argv[1]));
    lRobotarmController.run();
  }
  else
  {
    std::cout << "Invalid usage, expected form ./kinematics [webcamId]" << std::endl;
  }

  return 0;
}
