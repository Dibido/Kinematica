#include "MatrixFunctions.h"

MatrixFunctions::MatrixFunctions()
{
}

Matrix<double, 2, 3> MatrixFunctions::computeJacobi(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas) const
{
  Matrix<double, 2, 3> lJacobiMatrix;

  // Convert given theta's to radians, as cos/sin functions require radians
  double lTheta1 = degreesToRadians(aThetas[0][0]);
  double lTheta2 = degreesToRadians(aThetas[1][0]);
  double lTheta3 = degreesToRadians(aThetas[2][0]);

  // Retrieving sidelengths from given matrix (for readability)
  double lSide1 = aSidelengths[0][0];
  double lSide2 = aSidelengths[1][0];
  double lSide3 = aSidelengths[2][0];

  // Computing X row
  lJacobiMatrix[0][0] = (lSide1 * cos(lTheta1)) + (lSide2 * cos(lTheta1 + lTheta2)) + (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));
  lJacobiMatrix[0][1] = 0 + (lSide2 * cos(lTheta1 + lTheta2)) + (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));
  lJacobiMatrix[0][2] = 0 + 0 + (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));

  // Computing Y row
  lJacobiMatrix[1][0] = -(lSide1 * sin(lTheta1)) - (lSide2 * sin(lTheta1 + lTheta2)) - (lSide3 * sin(lTheta1 + lTheta2 + lTheta3));
  lJacobiMatrix[1][1] = 0 - (lSide2 * sin(lTheta1 + lTheta2)) - (lSide3 * sin(lTheta1 + lTheta2 + lTheta3));
  lJacobiMatrix[1][2] = 0 - 0 - (lSide3 * sin(lTheta1 + lTheta2 + lTheta3));

  return lJacobiMatrix;
}

Matrix<double, 2, 1> MatrixFunctions::computeEndEffector(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas) const
{
  Matrix<double, 2, 1> lEffectorMatrix;

  // Convert given theta's to radians, as cos/sin functions require radians
  double lTheta1 = degreesToRadians(aThetas[0][0]);
  double lTheta2 = degreesToRadians(aThetas[1][0]);
  double lTheta3 = degreesToRadians(aThetas[2][0]);

  // Calculating X
  lEffectorMatrix[0][0] = (aSidelengths[0][0] * sin(lTheta1)) + (aSidelengths[1][0] * sin(lTheta1 + lTheta2)) + (aSidelengths[2][0] * sin(lTheta1 + lTheta2 + lTheta3));

  // Calculating Y
  lEffectorMatrix[0][1] = (aSidelengths[0][0] * cos(lTheta1)) + (aSidelengths[1][0] * cos(lTheta1 + lTheta2)) + (aSidelengths[2][0] * cos(lTheta1 + lTheta2 + lTheta3));

  return lEffectorMatrix;
}

double MatrixFunctions::degreesToRadians(double aAngle) const
{
  return aAngle * (M_PI / 180.0);
}

Matrix<double, 3, 2> MatrixFunctions::computeInverseJacobi(Matrix<double, 2, 3> aOriginalJacobi) const
{
  return (aOriginalJacobi.transpose() * (aOriginalJacobi * aOriginalJacobi.transpose()).inverse());
}

bool MatrixFunctions::areThetasInRange(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges) const
{
  bool lReturn = true;
  for (size_t i = 0; i < aThetas.getRows(); ++i)
  {
    // If theta not within the given boundaries
    if (!(aThetas[i][0] >= aThetaRanges[i][0] && aThetas[i][0] <= aThetaRanges[i][1]))
    {
      lReturn = false;
    }
  }

  return lReturn;
}

void MatrixFunctions::randomizeThetas(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges) const
{
  for (size_t i = 0; i < aThetas.getRows(); ++i)
  {
    aThetas[i][0] = (aThetaRanges[i][1] - aThetaRanges[i][0]) * ((double)rand() / (double)RAND_MAX) + aThetaRanges[i][0];
  }
}