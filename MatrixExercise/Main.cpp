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

#define UNIT_TEST false

MatrixFunctions gMatrixFunctions;

Matrix<double, 3, 1> gSidelengths = {{{14.605}},
                                     {{18.733}},
                                     {{8.573}}};

Matrix<double, 3, 1> gThetas = {{{0}},
                                {{0}},
                                {{90}}};

// The allowed theta ranges, row 1 contains min max of theta 1 in degrees etc.
Matrix<double, 3, 2> gThetaRanges = {{{-30}, {90}}, {{0}, {135}}, {{-90}, {90}}};

Matrix<double, 3, 1> gGoalConfiguration =  {{{-25}},
                                           {{11}},
                                           {{31}}};

// Goal effector based on a valid gGoalConfiguration:
Matrix<double, 2, 1>
    gGoal = {{{gMatrixFunctions.computeEndEffector(gSidelengths, gGoalConfiguration)[0][0]}}, {{gMatrixFunctions.computeEndEffector(gSidelengths, gGoalConfiguration)[1][0]}}};

Matrix<double, 2, 1> gDeltaEffector = {{{0}}, {{0}}};

double gDeltaFactor = 0.1;

int main(int argc,
         char **argv)
{

  try
  {
    // In a real program we should test whether we should call the real main or run the unit_test_main
    if (UNIT_TEST)
    {
      return boost::unit_test::unit_test_main(&init_unit_test, argc, argv); // @suppress("Symbol is not resolved") // @suppress("Invalid arguments")
    }
    else
    {
      Matrix<double, 2, 1> lDetectedRobotarmBaseCoordinates;
      Matrix<double, 2, 1> lDetectedShapeCoordinates;
      Matrix<double, 2, 1> lDetectedBaseCoordinates;
      if (argc > 1)
      {
        Shapedetector shapeDetector;
        // Get the pixel/centimeter size using calibration
        double lCoordinateSystemConversionValue = shapeDetector.calibrateCoordinates(atoi(argv[1]));
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
        std::cout << "Converted robotarm base coordinates : " << lDetectedRobotarmBaseCoordinates <<  std::endl;
        std::cout << "Detected Base: " << lDetectedBaseCoordinates << std::endl;
      }
      else
      {
        std::cout << "Invalid usage, expected form ./kinematics [webcamId]" << std::endl;
        exit(0);
      }

      Matrix<double, 2, 1> lCalculatedEndEffector = gMatrixFunctions.computeEndEffector(gSidelengths, gThetas);

      bool foundValidConfiguration = false;
      while (!foundValidConfiguration)
      {
        int iterations = 1;
        // Run the inverse kinematics algorithm
        while (!equals(lCalculatedEndEffector, gGoal, 0.01))
        {
          // Calculate the Jacobi matrix
          Matrix<double, 2, 3> originalJacobi = gMatrixFunctions.computeJacobi(gSidelengths, gThetas);

          // Calculate (pseudo)inverse of Jacobi
          Matrix<double, 3, 2> inverseJacobi = gMatrixFunctions.computeInverseJacobi(originalJacobi);

          // Calculate delta end effector
          Matrix<double, 2, 1> deltaEffector = (gGoal - lCalculatedEndEffector) * gDeltaFactor;

          // Calculate the delta in theta's when moving delta effector
          Matrix<double, 3, 1> deltaThetas = inverseJacobi * deltaEffector;

          // Update new thetas
          gThetas += deltaThetas;

          // Calculate new position of the end effector with forward kinematics
          lCalculatedEndEffector = gMatrixFunctions.computeEndEffector(gSidelengths, gThetas);
        }

        iterations++;

        if (gMatrixFunctions.areThetasInRange(gThetas, gThetaRanges))
        {
          foundValidConfiguration = true;
          std::cout << "Found valid configuration in " << iterations << " iterations for endpoint:" << std::endl;
          std::cout << gGoal << std::endl;
          std::cout << "Configuration:" << std::endl;
          std::cout << gThetas << std::endl;
        }
        else
        {
          gMatrixFunctions.randomizeThetas(gThetas, gThetaRanges);

          // Calculate new position of the end effector with forward kinematics
          lCalculatedEndEffector = gMatrixFunctions.computeEndEffector(gSidelengths, gThetas);
        }
      }
    }
  }
  catch (std::exception &e)
  {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
