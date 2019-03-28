#ifndef MATRIXFUNCTIONS_H
#define MATRIXFUNCTIONS_H

#include <math.h>

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

};




#endif // MatrixFunctions.h