#ifndef MATRIXFUNCTIONS_H
#define MATRIXFUNCTIONS_H

#include <math.h>
#include <random>

#include "Matrix.hpp"

class MatrixFunctions
{
  public:
  MatrixFunctions();
  
  /**
   * @brief Construct a new Jacobi matrix based on current configuration
   * @param theta1 - First angle in degrees
   * @param theta2 - Second angle in degrees
   * @param theta3 - Third angle in degrees
   * @return A 2 * 3 sized Jacobi-matrix
   */
  Matrix<double, 2, 3> computeJacobi(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas) const;

  /**
   * @brief Determines position of end effector for a 3-DoF robot in 2D (x,y). Sidelenghts can be in any
   * measuring unit, the returned x,y will be based on given numbers. 
   * @param aSidelengths - Sidelengths
   * @param aThetas - Angles in degrees
   * @return A column-vector, containing X and Y of end effector ([0][0] contains x, [0][1] contains y)
   */
  Matrix<double, 2, 1> computeEndEffector(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas) const;
  
  /**
   * @brief Converts a given angle (in degrees) into radians
   * @return Corresponding radians. 
   */
  double degreesToRadians(double aAngle) const;

  /**
   * @brief Computes the pseudo-inverse of a jacobi matrix, uses the moore-penrose approach.
   * @return Pseudo inverse
   */
  Matrix<double, 3, 2> computeInverseJacobi(Matrix<double, 2, 3> aOriginalJacobi) const;

  /**
   * @brief Checks whether a set of theta's falls within the allowed ranges. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - Three different theta's in degrees
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees.
   * @return Returns true if the given theta's fall within the thetaranges.
   */
  bool areThetasInRange(Matrix<double, 3, 1>& aThetas, Matrix<double, 3, 2>& aThetaRanges) const;

  /**
   * @brief Randomizes a set of given theta's to values within a given range. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - The given theta's, these are passed by reference and WILL BE ALTERED
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees, the theta's will be randomized within this range.
   */
  void randomizeThetas(Matrix<double, 3, 1>& aThetas, Matrix<double, 3, 2>& aThetaRanges) const;
};




#endif // MatrixFunctions.h