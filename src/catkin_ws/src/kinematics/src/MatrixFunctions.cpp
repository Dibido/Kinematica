#include "MatrixFunctions.h"

Matrix<double, 2, 3> MatrixFunctions::computeJacobi(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas)
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

Matrix<double, 2, 1> MatrixFunctions::computeEndEffector(Matrix<double, 3, 1> aSidelengths, Matrix<double, 3, 1> aThetas)
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

double MatrixFunctions::degreesToRadians(double aAngle)
{
  return aAngle * (M_PI / 180.0);
}

Matrix<double, 3, 2> MatrixFunctions::computeInverseJacobi(Matrix<double, 2, 3> aOriginalJacobi)
{
  return (aOriginalJacobi.transpose() * (aOriginalJacobi * aOriginalJacobi.transpose()).inverse());
}

bool MatrixFunctions::areThetasInRange(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges)
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

void MatrixFunctions::randomizeThetas(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges)
{
  for (size_t i = 0; i < aThetas.getRows(); ++i)
  {
    aThetas[i][0] = (aThetaRanges[i][1] - aThetaRanges[i][0]) * ((double)rand() / (double)RAND_MAX) + aThetaRanges[i][0];
  }
}

std::pair<bool, Matrix<double, 3, 1>> MatrixFunctions::computeConfiguration(Matrix<double, 2, 1> &aGoal, Matrix<double, 3, 1>& aSidelengths, Matrix<double, 3, 1> aCurrentConfiguration, Matrix<double, 3, 2> &aThetaRanges, int aMaxIterations)
{
  std::pair<bool, Matrix<double, 3, 1>> lReturnPair;
 
  bool lFoundValidConfiguration = false;

  Matrix<double, 3, 1> lCurrentConfiguration = aCurrentConfiguration;

  Matrix<double, 2, 1> lCalculatedEndEffector = MatrixFunctions::computeEndEffector(aSidelengths, lCurrentConfiguration);

  // Factor which determines stepsize between pose and goal, 0.1 has proven to work just fine.
  double lDeltaEffectorFactor = 0.1;

  int lIterations = 0;

  while (!lFoundValidConfiguration && lIterations < aMaxIterations)
  {
    // Run the inverse kinematics algorithm, 0.01 being the procession. If goal is 90.000 degrees, >= 89.99 and <= 90.01 will be close enough
    while (!equals(lCalculatedEndEffector, aGoal, 0.01))
    {
      // Calculate the Jacobi matrix
      Matrix<double, 2, 3> lOriginalJacobi = MatrixFunctions::computeJacobi(aSidelengths, lCurrentConfiguration);

      // Calculate (pseudo)inverse of Jacobi
      Matrix<double, 3, 2> inverseJacobi = MatrixFunctions::computeInverseJacobi(lOriginalJacobi);

      // Calculate delta end effector based on 
      Matrix<double, 2, 1> deltaEffector = (aGoal - lCalculatedEndEffector) * lDeltaEffectorFactor;

      // Calculate the delta in theta's when moving delta effector
      Matrix<double, 3, 1> deltaThetas = inverseJacobi * deltaEffector;

      // Update new thetas
      lCurrentConfiguration += deltaThetas;

      // Calculate new position of the end effector with forward kinematics
      lCalculatedEndEffector = MatrixFunctions::computeEndEffector(aSidelengths, lCurrentConfiguration);
    }

    lIterations++;

    if (MatrixFunctions::areThetasInRange(lCurrentConfiguration, aThetaRanges))
    {
      lFoundValidConfiguration = true;
    }
    else
    {
      MatrixFunctions::randomizeThetas(lCurrentConfiguration, aThetaRanges);
      
      // Calculate new position of the end effector with forward kinematics
      lCalculatedEndEffector = MatrixFunctions::computeEndEffector(aSidelengths, lCurrentConfiguration);
    }
  }

  lReturnPair.first = lFoundValidConfiguration;
  lReturnPair.second = lCurrentConfiguration;

  return lReturnPair;
}

double MatrixFunctions::calculateBaseAngle(Matrix<double, 2, 1> aBase, Matrix<double, 2, 1> aTarget)
{
  if(!(aTarget[0][0] < aBase[0][0]))
  {
    throw std::logic_error("calculateBaseAngle preconditions are not met, aTarget.x must be smaller then aBase.x for valid calculations");
  }

  Matrix<double, 2, 1> lDeltaTargetBase = aTarget - aBase;

  std::cout << "aBase : " << aBase << " : aTarget : " << aTarget << std::endl;
  std::cout << "DeltaTargetBase : " << lDeltaTargetBase << std::endl;
  
  // Tangens of corner = delta Y / delta X
  double lTanAngle = std::abs(lDeltaTargetBase[1][0]) / std::abs(lDeltaTargetBase[0][0]);

  std::cout << "TanAngle : " << lTanAngle << std::endl;

  // If delta Y is negative, we want the angle to be negative aswell.
  double lReturnAngle = (lDeltaTargetBase[1][0] > 0) ? -atan(lTanAngle) : atan(lTanAngle);

  // Convert to degrees
  lReturnAngle *=  180/M_PI;

  return lReturnAngle;
}

double MatrixFunctions::calcDistance(Matrix<double, 2, 1> aPoint1, Matrix<double, 2, 1> aPoint2)
{
  Matrix<double, 2, 1> lDelta = {{{aPoint1[0][0] - aPoint2[0][0]}}, {{aPoint1[1][0] - aPoint2[1][0]}}};
  return sqrt(lDelta[0][0] * lDelta[0][0] + lDelta[1][0] * lDelta[1][0]);
}