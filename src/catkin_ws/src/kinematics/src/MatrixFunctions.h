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
double degreesToRadians(const double aAngle);

/**
   * @brief Computes the pseudo-inverse of a jacobi matrix, uses the moore-penrose approach.
   * @return Pseudo inverse
   */
Matrix<double, 3, 2> computeInverseJacobi(const Matrix<double, 2, 3>& aOriginalJacobi);

/**
   * @brief Checks whether a set of theta's falls within the allowed ranges. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - Three different theta's in degrees
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees.
   * @return Returns true if the given theta's fall within the thetaranges.
   */
bool areThetasInRange(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges);

/**
   * @brief Randomizes a set of given theta's to values within a given range. Ordering of aThetas should correspond with ordering of aThetaRanges
   * @param aThetas - The given theta's, these are passed by reference and WILL BE ALTERED
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees, the theta's will be randomized within this range.
   */
void randomizeThetas(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges);

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
std::pair<bool, Matrix<double, 3, 1>> computeConfiguration(Matrix<double, 2, 1> &aGoal, const Matrix<double, 3, 1> &aSidelengths, const Matrix<double, 3, 1>& aCurrentConfiguration, Matrix<double, 3, 2> &aThetaRanges, int aMaxIterations);

/**
   * @brief Calculates the angle the target has to the base. To more exact, the base point is considered to have 
   * a X-axis, one could draw this out by drawing a line horizontally over the midpoint of the. Then one could draw a line
   * from the base to the target. The angle between that line and the X-axis, is considered 'the angle'. 
   * @param aBase - Base point x,y
   * @param aTarget - Target point x,y NOTE: x of target MUST be smaller then x of base
   * @return double - Angle, when target is below the X-axis of the base this angle is negative 
   * (reason behind it: Base can move from -90 to 90)
   */
double calculateBaseAngle(Matrix<double, 2, 1> aBase, Matrix<double, 2, 1> aTarget);

/**
   * @brief Calculates the minimum distance between two points (pythagoras)
   * @param aPoint1 - First point 
   * @param aPoint2 - Second point
   * @return The distance between both points
   */
double calcDistance(Matrix<double, 2, 1> aPoint1, Matrix<double, 2, 1> aPoint2);
}; // namespace MatrixFunctions

#endif // MatrixFunctions.h