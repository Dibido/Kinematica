#ifndef SHAPE_H_
#define SHAPE_H_

#include "Matrix.hpp"

struct Shape
{
  Matrix<double, 2, 1> mCenterPoint;
  Matrix<double, 4, 2> mBoundingRect;
  double mShapeWidth;

  // To-Do: add height/width of shape?
};


#endif /* SHAPE_H_ */