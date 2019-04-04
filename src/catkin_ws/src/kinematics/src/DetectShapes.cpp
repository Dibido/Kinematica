#include "Shapedetector.h"

void Shapedetector::detectSquares(std::vector<Mat> aContours)
{
  for (size_t i = 0; i < aContours.size(); i++)
  {
    double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
    approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == SQUARE_CORNERCOUNT)
    {
      if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
      {
        //Check if it is a square
        Rect boundedRect = boundingRect(mCurrentContours.at(i));
        float ratio = (float)boundedRect.width / (float)boundedRect.height;
        if(ratio > mMinSquareRatio && ratio < mMaxSquareRatio)
        {
          drawShapeContours(mDisplayImage, mCurrentContours.at(i));
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
          mCurrentShapeCount++;
          drawShapeContours(mDisplayImage, mCurrentContours.at(i));
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
        }
        else
        {
          std::cout << "Min Ratio : " << mMinSquareRatio << " Max Ratio : " << mMaxSquareRatio << std::endl;
          std::cout << "Curent Ratio : " << ratio << std::endl;
        }
      }
    }
  }
}

void Shapedetector::detectRectangles(std::vector<Mat> aContours)
{
  for (size_t i = 0; i < aContours.size(); i++)
  {
    double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
    approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == SQUARE_CORNERCOUNT)
    {
      if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
      {
        mCurrentShapeCount++;
        drawShapeContours(mDisplayImage, mCurrentContours.at(i));
        setShapeValues(mDisplayImage, mCurrentContours.at(i));
      }
    }
  }
}

void Shapedetector::detectTriangles(std::vector<Mat> aContours)
{
  for (size_t i = 0; i < aContours.size(); i++)
  {
    double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
    approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == TRIANGLE_CORNERCOUNT)
    {
      if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
      {
        mCurrentShapeCount++;
        drawShapeContours(mDisplayImage, mCurrentContours.at(i));
        setShapeValues(mDisplayImage, mCurrentContours.at(i));
      }
    }
  }
}

void Shapedetector::detectCircles(std::vector<Mat> aContours)
{
  for (size_t i = 0; i < aContours.size(); i++)
  {
    double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
    approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height > 5)
    {
      if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
      {
        mCurrentShapeCount++;
        drawShapeContours(mDisplayImage, mCurrentContours.at(i));
        setShapeValues(mDisplayImage, mCurrentContours.at(i));
      }
    }
  }
}

void Shapedetector::detectHalfCircles(std::vector<Mat> aContours)
{
  for (size_t i = 0; i < aContours.size(); i++)
  {
    double epsilon = mEpsilonMultiply * arcLength(aContours.at(i), true);
    approxPolyDP(aContours.at(i), mApproxImage, epsilon, true);
    if (mApproxImage.size().height == 5)
    {
      if (contourSizeAllowed(aContours.at(i), mMinContourSize, mMaxContourSize))
      {
        //Check for half circle
        Rect boundedRect = boundingRect(aContours.at(i));
        double shapeArea = contourArea(aContours.at(i));
        float squareArea = (float)boundedRect.width * (float)boundedRect.height;
        double shapePercentage = (100.0f * ((float)shapeArea / (float)squareArea));
        if(shapePercentage > mMinHalfCirclePercentage && shapePercentage < mMaxHalfCirclePercentage)
        {
          mCurrentShapeCount++;
          drawShapeContours(mDisplayImage, mCurrentContours.at(i));
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
        }
      }
    }
  }
}

bool Shapedetector::contourSizeAllowed(Mat aContour, double aMinContourSize, double aMaxContourSize) const
{
  return (contourArea(aContour) > aMinContourSize && contourArea(aContour) < aMaxContourSize);
}

std::vector<Mat> Shapedetector::detectShape(SHAPES aShape, Mat aShapeMask)
{
  findContours(aShapeMask, mCurrentContours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
  removeCloseShapes(mCurrentContours);
  switch (aShape)
  {
    case SHAPES::ALL_SHAPES:
    {
      for (size_t i = 0; i < mCurrentContours.size(); i++)
      {
        double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
        approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
        if(contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
        {
          mCurrentShapeCount++;
          drawShapeContours(mDisplayImage, mCurrentContours.at(i));
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
        }
      }
      break;
    }
    case SHAPES::SQUARE:
    {
      detectSquares(mCurrentContours);
      break;
    }
    case SHAPES::RECTANGLE:
    {
      detectRectangles(mCurrentContours);
      break;
    }
    case SHAPES::TRIANGLE:
    {
      detectTriangles(mCurrentContours);
      break;
    }
    case SHAPES::CIRCLE:
    {
      detectCircles(mCurrentContours); 
      break;
    }
    case SHAPES::HALFCIRCLE:
    {
      detectHalfCircles(mCurrentContours);
      break;
    }
    case SHAPES::UNKNOWNSHAPE:
    {
      std::cout << "ERROR - Unknown shape" << std::endl;
      break;
    }
  }
  return mCurrentContours;
}

void Shapedetector::removeCloseShapes(std::vector<Mat> &aContours) const
{
  Point currentCenter;
  Point compareCenter;
  for (size_t i = 0; i < aContours.size(); i++)
  {
    currentCenter = getContourCenter(aContours.at(i));
    //Remove duplicates
    for (size_t j = 0; j < aContours.size(); j++)
    {
      if (j != i) // Not the same shape
      {
        compareCenter = getContourCenter(aContours.at(j));
        int Xdiff = abs(currentCenter.x - compareCenter.x);
        int Ydiff = abs(currentCenter.y - compareCenter.y);
        //Shape is too close
        if (Xdiff <= mContourCenterMargin && Ydiff <= mContourCenterMargin)
        {
          aContours.erase(aContours.begin() + j);
        }
      }
    }
  }
}

void Shapedetector::setShapeValues(Mat aImage, Mat aContour)
{
  Point currentCenter = getContourCenter(aContour);
  const std::string xPosString = std::string("X: " + std::to_string(currentCenter.x));
  const std::string yPosString = std::string("Y: " + std::to_string(currentCenter.y));
  const std::string areaString = std::string("A: " + std::to_string((int)contourArea(aContour)));
  Point2f lVertices[4];

  // Place values in the image
  putText(aImage, xPosString, Point(currentCenter.x, currentCenter.y), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(255, 255, 255), 1);
  putText(aImage, yPosString, Point(currentCenter.x, currentCenter.y + mTextOffset), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(255, 255, 255), 1);
  putText(aImage, areaString, Point(currentCenter.x, currentCenter.y + (mTextOffset * 2)), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(255, 255, 255), 1);

  // Print to stdout
  std::cout << "\tShape location:\t" << xPosString << "\t" << yPosString << "\t" << areaString << std::endl;

  // Set the shape position return value
  mShapePosition = {{{double(currentCenter.x)}}, {{double(currentCenter.y)}}};
  // Create minarearect
  RotatedRect lRotatedRect = minAreaRect(aContour);
  // Rect lBoundingRect = boundingRect(lRotatedRect.boundingRect;
  rectangle(mDisplayImage, lRotatedRect.boundingRect(), Scalar(0,0,255));
  double lShapeAngle = lRotatedRect.angle;
  if (lRotatedRect.size.width < lRotatedRect.size.height) {
    lShapeAngle = lShapeAngle - 90;
  }

  std::cout << "lShapeAngle : " << lShapeAngle << std::endl;
  
  // Get the smallest side
  double lMinDistance = DBL_MAX;
  double lMaxDistance = 0;
  lRotatedRect.points(lVertices);
  for (int i = 0; i < 4; i++)
  {
    line(mDisplayImage, lVertices[i], lVertices[(i+1)%4], Scalar(0,0,255));
    double lDistance = (double)cv::norm(lVertices[i] - lVertices[(i+1)%4]);
    if(lDistance < lMinDistance)
    {
      lMinDistance = (double)lDistance;
    }
    if(lDistance > lMaxDistance)
    {
      lMaxDistance = (double)lDistance;
    }
  }
  circle(mDisplayImage, getContourCenter(aContour), 3, Scalar(0,0,255), -1, 8, 0 );
  // Set the bounding rect corners in the return value
  for (int i = 0; i < 4; i++)
  {
    mShapeBoundingRect.at(0,i) = lVertices[i].x;
    mShapeBoundingRect.at(1,i) = lVertices[i].y;
  }
  // Set the shape angle
  mShapeAngle = lShapeAngle;
  // Set the shape width and height
  mShapeWidth = lMinDistance;
  mShapeHeight = lMaxDistance;
}

void Shapedetector::drawShapeContours(Mat aImage, Mat aContour)
{
  drawContours(aImage, aContour, -1, Scalar(0, 255, 0), 3);
}

Point Shapedetector::getContourCenter(Mat aContour)
{
  //Calculate center
  Moments currentmoments = moments(aContour);
  int centerX = (int)(currentmoments.m10 / currentmoments.m00);
  int centerY = (int)(currentmoments.m01 / currentmoments.m00);
  return Point(centerX, centerY);
}