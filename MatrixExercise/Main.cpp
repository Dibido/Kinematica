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

#define UNIT_TEST false

MatrixFunctions gMatrixFunctions;

Matrix<double, 3, 1> gSidelengths = {{{14.605}},
                                     {{18.733}},
                                     {{8.573}}};

Matrix<double, 3, 1> gThetas = {{{0}},
                                {{0}},
                                {{90}}};

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

      Matrix<double, 2, 1> lDetectedShapeCoordinates;
      if (argc > 1)
      {
        Shapedetector shapeDetector;
        // Get target coordinates
        lDetectedShapeCoordinates = shapeDetector.webcamMode(atoi(argv[1]));
        std::cout << "Detected : " << lDetectedShapeCoordinates << std::endl;
      }

      Matrix<double, 2, 1> lGoal = {{{5}}, {{2}}};
      Matrix<double, 2, 1> lCurrentEndEffector = {{{5}}, {{15}}};

      Matrix<double, 2, 1> lCalculatedEndEffector = gMatrixFunctions.computeEndEffector(gSidelengths, gThetas);
      std::cout << "Effector: " << lCalculatedEndEffector << std::endl;

      // Run the inverse kinematics algorithm
      while (!equals(lCalculatedEndEffector, lGoal))
      {
        // Calculate the Jacobi matrix
        // Get the 2 * 3 Matrix
        Matrix<double, 2, 3> originalJacobi = gMatrixFunctions.computeJacobi(gSidelengths, gThetas);

        // Create the 3 * 3 Matrix using transpose
        Matrix<double, 3, 2> JacobiTranspose = originalJacobi.transpose();
        Matrix<double, 2, 2> Jacobi = originalJacobi * JacobiTranspose;
        // Invert the matrix
        Matrix<double, 2, 2> InvertedJacobi = Jacobi.inverse();
        Matrix<double, 3, 2> PartialInvertedJacobi = JacobiTranspose * InvertedJacobi;
        // Calculate end effector delta
        // Calculate theta delta
        // Calculate new theta delta
        // Compute new end effector
        lCalculatedEndEffector = gMatrixFunctions.computeEndEffector(gSidelengths, gThetas);
        break;
      }
    }
  }
  catch (std::exception &e)
  {
    std::cout << e.what() << std::endl;
  }
  return 0;
}
