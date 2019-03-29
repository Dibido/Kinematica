#ifndef MATRIXFUNCTIONS_H
#define MATRIXFUNCTIONS_H

#include <math.h>
#include <random>

#include "Matrix.hpp"

namespace MatrixFunctions
{
  /**
   * @brief Construct a new Jacobi matrix based on current configuration
   * @param theta1 - First angle in degrees
   * @param theta2 - Second angle in degrees
   * @param theta3 - Third angle in degrees
   * @return A 2 * 3 sized Jacobi-matrix
   */
  Matrix<double, 2, 3> computeJacobi(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas);

  /**
   * @brief Determines position of end effector for a 3-DoF robot in 2D (x,y). Sidelenghts can be in any
   * measuring unit, the returned x,y will be based on given numbers. 
   * @param aSidelengths - Sidelengths
   * @param aThetas - Angles in degrees
   * @return A column-vector, containing X and Y of end effector ([0][0] contains x, [0][1] contains y)
   */
  Matrix<double, 2, 1> computeEndEffector(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas);
  
  /**
   * @brief Converts a given angle (in degrees) into radians
   * @return Corresponding radians. 
   */
  double degreesToRadians(double aAngle);

  /**
   * @brief Computes the pseudo-inverse of a jacobi matrix, uses the moore-penrose approach.
   * @return Pseudo inverse
   */
  Matrix<double, 3, 2> computeInverseJacobi(Matrix<double, 2, 3> aOriginalJacobi);

  /**
   * @brief Checks whether a set of theta's falls within the allowed ranges. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - Three different theta's in degrees
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees.
   * @return Returns true if the given theta's fall within the thetaranges.
   */
  bool areThetasInRange(Matrix<double, 3, 1>& aThetas, Matrix<double, 3, 2>& aThetaRanges);

  /**
   * @brief Randomizes a set of given theta's to values within a given range. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - The given theta's, these are passed by reference and WILL BE ALTERED
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees, the theta's will be randomized within this range.
   */
  void randomizeThetas(Matrix<double, 3, 1>& aThetas, Matrix<double, 3, 2>& aThetaRanges);

  /**
   * @brief Inverse kinematics, attempts to find a valid configuration for a certain x,y position.  
   * @param aGoal - The goal point [0][0] = X and [1][0] = Y.
   * @param aSidelength -  Sidelengths of the arm.
   * @param aCurrentConfiguration - Current configuration of theta's, used to calculate starting point.
   * @param aThetaRanges - The allowed theta ranges for the configuration to be considered valid. [0][0] = Min of first theta, [0][1] = Max of first theta. Corresponds
   * with index of aCurrentConfiguration.
   * @param aMaxIterations - The max number of attempts that will take place to find the goal. 
   * @return A pair, first element of pair indicates whether a valid configuration was found. Second element is the corresponding configuration,
   * if the first element returns false, the second element should be ignored as it contains no useful information.
   */
  std::pair<bool, Matrix<double, 3, 1>> computeConfiguration(Matrix<double, 2, 1>& aGoal, Matrix<double, 3, 1>& aSidelengths, Matrix<double, 3, 1> aCurrentConfiguration, Matrix<double, 3, 2>& aThetaRanges, int aMaxIterations);
};

#endif // MatrixFunctions.h