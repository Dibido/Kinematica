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
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <stdlib.h>

#define UNIT_TEST false

int main( 	int argc,
			char** argv)
{
	try
	{
		// In a real program we should test whether we should call the real main or run the unit_test_main
		if(UNIT_TEST)
		{
			return boost::unit_test::unit_test_main( &init_unit_test, argc, argv ); // @suppress("Symbol is not resolved") // @suppress("Invalid arguments")
		}
		else
		{
      // Read target from input
      
      // Detect target

      // Get target coordinates

      // Calculate new arm angles
			double l1 = 5;
			double theta1 = 90;
			double l2 = 10;
			double theta2 = 120;
			double l3 = 15;
			double theta3 = 180;

			Matrix<double, 2 , 1> lGoal = {{{5}}, {{2}}};
			Matrix<double, 2 , 1> lCurrentEndEffector = {{{5}}, {{15}}};
			Matrix<double, 2 , 1> lCalculatedEndEffector = {{{5}}, {{15}}};
			// Run the inverse kinematics algorithm
			while(!equals(lCalculatedEndEffector, lGoal))
			{
				// Calculate the Jordan matrix
				// Get the 2 * 3 Matrix
				Matrix<double, 2, 3> originalJordan;
				// X row
				originalJordan.at(0, 0) = (l1 * cos(theta1 * (M_PI / 180.0))) + (l2 * cos((theta1 + theta2) * (M_PI / 180.0))) + (l3 * cos((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				originalJordan.at(0, 1) = 0 + (l2 * cos((theta1 + theta2) * (M_PI / 180.0))) + (l3 * cos((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				originalJordan.at(0, 2) = 0 + 0 + (l3 * cos((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				// Y row
				originalJordan.at(1, 0) = (l1 * sin(theta1 * (M_PI / 180.0))) - (l2 * sin((theta1 + theta2) * (M_PI / 180.0))) - (l3 * sin((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				originalJordan.at(1, 1) = 0 - (l2 * sin((theta1 + theta2) * (M_PI / 180.0))) - (l3 * sin((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				originalJordan.at(1, 2) = 0 - 0 - (l3 * sin((theta1 + theta2 + theta3) * (M_PI / 180.0)));
				// Create the 3 * 3 Matrix using transpose
				Matrix<double, 3, 2> JordanTranspose = originalJordan.transpose();
				Matrix<double, 2, 2> Jordan = originalJordan * JordanTranspose;
				// Invert the matrix
				Matrix<double, 2, 2> InvertedJordan = Jordan.inverse();
				Matrix<double, 3, 2> PartialInvertedJordan = JordanTranspose * InvertedJordan;
				// Calculate end effector delta
				// Calculate theta delta
				// Calculate new theta delta
				// Compute new end effector
				break;
			}
		}
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
	return 0;
}
