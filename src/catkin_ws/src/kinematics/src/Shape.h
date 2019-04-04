#ifndef SHAPE_H_
#define SHAPE_H_

#include "Matrix.hpp"

struct Shape
{
  Matrix<double, 2, 1> mCenterPoint;
  Matrix<double, 2, 4> mBoundingRect;
  double mShapeWidth;
  double mShapeHeight;
  double mShapeAngle;
};


#endif /* SHAPE_H_ */