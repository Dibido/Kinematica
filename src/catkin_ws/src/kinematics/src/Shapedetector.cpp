// Local
#include "Shapedetector.h"

// Constructor
Shapedetector::Shapedetector()
{
  initializeValues();
}

void Shapedetector::setImage(Mat aImage)
{
  // Store origininal image
  mOriginalImage = aImage;
  mOriginalImage.copyTo(mDisplayImage);
  mOriginalImage.copyTo(mTresholdImage);

  // Convert to necessary formats
  cvtColor(mOriginalImage, mGreyImage, CV_BGR2GRAY);
  cvtColor(mOriginalImage, mHSVImage, CV_BGR2HSV);
}

void Shapedetector::reset()
{
  // Reload frames
  mOriginalImage.copyTo(mDisplayImage);
  mOriginalImage.copyTo(mTresholdImage);
  cvtColor(mOriginalImage, mHSVImage, CV_BGR2HSV);
  mCurrentShapeCount = 0; // Reset shape count
}

void Shapedetector::initializeValues()
{
  // Set shape size to find
  mFindShapeWidth = 0.0;
  mFindShapeHeight = 0.0;
  
  // Set shape position
  mShapePosition = {{{0}}, {{0}}};
  mShapeMinDistance = DBL_MAX;

  // Set shape and color
  mCurrentColor = COLORS::UNKNOWNCOLOR;
  mCurrentShape = SHAPES::UNKNOWNSHAPE;

  // Set the calibration variables
  mContrastSliderValue = 0;
  mBlurSliderValue = 0;
  mNoiseSliderValue = 0;
  mMinRatioSliderValue = 75;
  mMaxRatioSliderValue = 125;

  mMinCalibrationHue = 0;
  mMaxCalibrationHue = 180;
  mMinCalibrationSaturation = 0;
  mMaxCalibrationSaturation = 255;
  mMinCalibrationValue = 0;
  mMaxCalibrationValue = 255;

  mCalibrationHueRange = 180;
  mCalibrationSaturationRange = 255;
  mCalibrationValueRange = 255;

  mContrastSliderRange = 200;
  mBlurSliderRange = 31; // must be an odd value
  mNoiseSliderRange = 50;
  mMinRatioSliderRange = 150;
  mMaxRatioSliderRange = 150;

  // Window size
  mScreenDrawWidth = 600;
  mScreenDrawHeight = mScreenDrawWidth * 1080 / 1920;

  // Set the blur variables
  mGaussianKernelsize = Size(3, 3);

  // Set the treshold variables
  mMinTreshold = 120;
  mMaxTreshold = 255;
  mTresholdType = ThresholdTypes::THRESH_BINARY;

  mMinSquareRatio = 0.60;
  mMaxSquareRatio = 1.40;

  // Set the Contours variables
  mContourCenterMargin = 30;
  mCurrentShapeCount = 0;
  mEpsilonMultiply = 0.03;
  mMinContourSize = 1000.0;
  mMaxContourSize = 30000.0;
  mMinBaseContourSize = 3000.0;
  mMaxBaseContourSize = 30000.0;
  mMinHalfCirclePercentage = 50.0;
  mMaxHalfCirclePercentage = 72.0;
  mTextOffset = 20;
  mTextSize = 0.5;

  // Set the timer variables
  mTimeXOffset = 20;
  mTimeYOffset = 20;

  // Set the color limits for color detection [0] = Min, [1] = Max
  mBlueLimits[0] = Scalar(70, 0, 0);
  mBlueLimits[1] = Scalar(95, 80, 45);

  mGreenLimits[0] = Scalar(75, 0, 0);
  mGreenLimits[1] = Scalar(125, 255, 50);

  mRedLimits[0] = Scalar(0, 0, 70);
  mRedLimits[1] = Scalar(50, 85, 255);

  mRedLimits[2] = Scalar(170, 60, 60);
  mRedLimits[3] = Scalar(180, 255, 255);

  mBlackLimits[0] = Scalar(0, 0, 0);
  mBlackLimits[1] = Scalar(255, 100, 100);

  mYellowLimits[0] = Scalar(25, 60, 60);
  mYellowLimits[1] = Scalar(45, 255, 255);

  mWhiteLimits[0] = Scalar(10, 30, 20); // Is woodcolor instead of white
  mWhiteLimits[1] = Scalar(24, 255, 255);
}

// Destructorclean
Shapedetector::~Shapedetector()
{
}

bool Shapedetector::parseSpec(const std::string &aShapeCommand)
{
  bool result = true;

  // Parse command
  std::size_t lDelimiterPos = aShapeCommand.find(' ');
  std::string shapeStr = aShapeCommand.substr(0, lDelimiterPos);
  std::size_t lSecondDelimiterPos = aShapeCommand.find(' ', lDelimiterPos + 1);
  mFindShapeWidth = std::stod(aShapeCommand.substr(lDelimiterPos + 1, lSecondDelimiterPos));
  std::size_t lThirdDelimiterPos = aShapeCommand.find(' ', lSecondDelimiterPos + 1);
  mFindShapeHeight = std::stod(aShapeCommand.substr(lSecondDelimiterPos + 1, lThirdDelimiterPos));
  std::string colorStr = aShapeCommand.substr(lThirdDelimiterPos + 1);

  mCurrentColor = StringToColor(colorStr); // convert string to enum
  if (mCurrentColor == COLORS::UNKNOWNCOLOR)
  {
    std::cout << "Error: unkown color entered" << std::endl;
    result = false;
  }

  mCurrentShape = StringToShape(shapeStr); // convert string to enum
  if (mCurrentShape == SHAPES::UNKNOWNSHAPE)
  {
    std::cout << "Error: unkown shape entered" << std::endl;
    result = false;
  }

  if (result)
  {
    mCurrentShapeCommand = aShapeCommand;
  }

  return result;
}

bool Shapedetector::showImages()
{
  bool keyPressed = false;

  reset();     // reset images and values
  recognize(); // run algorithm

  // Show images
  imshow("Original", mOriginalImage);
  imshow("Color", mMaskImage);
  imshow("Result", mDisplayImage);

  imshow("Brightness", mBrightenedRgbImage);
  imshow("Blur", mBlurredImage);

  int pressedKey = waitKey(30);
  if (pressedKey == 27) // ESC key
  {
    destroyAllWindows();
    keyPressed = true;
  }

  return keyPressed;
}

void Shapedetector::draw()
{
  // Show original
  namedWindow("Original", WINDOW_NORMAL);
  moveWindow("Original", 0, 0);
  resizeWindow("Original", mScreenDrawWidth, mScreenDrawHeight);

  // Show mask (optional)
  namedWindow("Color", WINDOW_NORMAL);
  moveWindow("Color", mScreenDrawWidth, 0);
  resizeWindow("Color", mScreenDrawWidth, mScreenDrawHeight);

  // Show result
  namedWindow("Result", WINDOW_NORMAL);
  moveWindow("Result", mScreenDrawWidth * 2, 0);
  resizeWindow("Result", mScreenDrawWidth, mScreenDrawHeight);

  // Sliders
  namedWindow("Sliders");
  createTrackbar("Brightness", "Sliders", &mContrastSliderValue, mContrastSliderRange, onChange, this);
  createTrackbar("Blur\t\t", "Sliders", &mBlurSliderValue, mBlurSliderRange, onChange, this);
  createTrackbar("Noise\t\t", "Sliders", &mNoiseSliderValue, mNoiseSliderRange, onChange, this);
  createTrackbar("minRatio\t\t", "Sliders", &mMinRatioSliderValue, mMinRatioSliderRange, onChange, this);
  createTrackbar("maxRatio\t\t", "Sliders", &mMaxRatioSliderValue, mMaxRatioSliderRange, onChange, this);
  moveWindow("Sliders", 0, mOriginalImage.rows + 10);

  const int sliderWidth = 500;
  Mat emptyMatrix = Mat::zeros(1, sliderWidth, CV_8U);
  imshow("Sliders", emptyMatrix); // put an empty matrix in this window to prevent errors
}

void Shapedetector::printDetectionData()
{
  std::cout << std::fixed << std::setprecision(2) << "\tT = " << ((double)mClockEnd - (double)mClockStart) << "\t\t";
  std::cout << std::to_string(mCurrentShapeCount) + " " + ShapeToString(mCurrentShape) << std::endl;
}

// Starts the detection algorithm
void Shapedetector::recognize()
{
  // Constrain/manipulate slider values
  mMinSquareRatio = mMinRatioSliderValue / 100.0;
  mMaxSquareRatio = mMaxRatioSliderValue / 100.0;

  if (mNoiseSliderValue == 0)
  {
    mNoiseSliderValue++; // noiseValue must be > 0
  }
  if (mBlurSliderValue % 2 == 0) // blur kernel size must be an odd value
  {
    mBlurSliderValue++;
  }

  // Start timer
  mClockStart = std::clock();

  //////////////////////
  // Apply filters
  //////////////////////

  // 1. Change brightness
  Mat brightenedBGRImage;
  Mat brightenedHSVImage;
  mOriginalImage.convertTo(brightenedBGRImage, -1, 1, mContrastSliderValue);
  cvtColor(brightenedBGRImage, brightenedHSVImage, COLOR_BGR2HSV);
  mBrightenedRgbImage = brightenedBGRImage;

  // 2. Blur
  Mat blurredHSVImage;
  Size blurValue = Size(mBlurSliderValue, mBlurSliderValue);
  GaussianBlur(brightenedHSVImage, blurredHSVImage, blurValue, 0);
  cvtColor(blurredHSVImage, mBlurredImage, COLOR_HSV2BGR); // save blurred output

  // 3. Filter color
  mMaskImage = detectColor(mCurrentColor, mOriginalImage);

  // 4. Remove noise
  Mat removedNoise = removeNoise(mMaskImage);

  // 5. Detect shapes
  detectShape(mCurrentShape, removedNoise);

  // Stop timer
  mClockEnd = std::clock();

  // Show recognition data in displayed image
  setShapeCommand(mDisplayImage);
  setTimeValue(mDisplayImage, mClockStart, mClockEnd);
  setShapeFound(mDisplayImage);
}

void Shapedetector::onChange(int, void *)
{
  // Slider callback function
}

void Shapedetector::setShapeCommand(Mat aImage)
{
  const std::string aShapeCommandString = "Shape :" + mCurrentShapeCommand;
  putText(aImage, aShapeCommandString, Point(mTimeXOffset, mTimeYOffset), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(0, 0, 0), 1);
}

void Shapedetector::setTimeValue(Mat aImage, std::clock_t aStartTime, std::clock_t aEndTime)
{
  double calcTime = ((double)aEndTime - (double)aStartTime);
  const std::string timeText = std::string("T:" + std::to_string(calcTime));
  putText(aImage, timeText, Point(mTimeXOffset, (mTimeYOffset * 2)), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(0, 0, 0), 1);
}

void Shapedetector::setShapeFound(Mat aImage)
{
  const std::string shapeCountText = std::to_string(mCurrentShapeCount) + " " + ShapeToString(mCurrentShape);
  putText(aImage, shapeCountText, Point(mTimeXOffset, (mTimeYOffset * 3)), FONT_HERSHEY_SIMPLEX, mTextSize, Scalar(0, 0, 0), 1);
}

Mat Shapedetector::removeNoise(Mat aImage)
{
  Mat result;
  Mat structure = getStructuringElement(MORPH_RECT, Size(mNoiseSliderValue, mNoiseSliderValue));
  morphologyEx(aImage, result, MORPH_OPEN, structure);
  return result;
}

Matrix<double, 2, 1> Shapedetector::calibrateRobotarmBase(double aCoordinateConversionValue, int aDeviceId)
{
  const double SHAPE_HEIGHT_SIZE_CM = 2.47;
  const double BASE_RADIUS_CM = 4.25;
  const double BASE_Y_DISTANCE_CM = (BASE_RADIUS_CM + (SHAPE_HEIGHT_SIZE_CM / 2));

  Matrix<double, 2, 1> lReturn;
  Point lBlockCenterPoint;
  Point lRobotarmBasePoint;

  initCamera(aDeviceId);
  Mat firstRetrievedFrame;
  mVidCap.grab();
  mVidCap.retrieve(firstRetrievedFrame);

  Point lBasePoint;
  while (true)
  {
    Mat retrievedFrame;
    mVidCap.grab();
    mVidCap.retrieve(retrievedFrame);
    mOriginalImage = retrievedFrame;
    reset();
    // Recognize calibration object
    // Filter color
    Mat colorMask;
    inRange(mOriginalImage, mRedLimits[0], mRedLimits[1], colorMask);
    imshow("colorMask", colorMask);
    // Find shape
    findContours(colorMask, mCurrentContours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    removeCloseShapes(mCurrentContours);
    for (size_t i = 0; i < mCurrentContours.size(); i++)
    {
      double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
      approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
      if (mApproxImage.size().height == SQUARE_CORNERCOUNT)
      {
        if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
        {
          // Get the center point
          // lBlockCenterPoint = getContourCenter(mCurrentContours.at(i));
          // Create a bounding rectangle
          RotatedRect lRotatedRect = minAreaRect(mCurrentContours.at(i));
          lBlockCenterPoint = lRotatedRect.center;
          Point2f vertices[4];
          lRotatedRect.points(vertices);
          for (int j = 0; j < 4; j++)
          {
            // Draw the rectangle
            line(mDisplayImage, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0));
          }
          // Calculate the base point using the block center point for reference
          lBasePoint = Point(lBlockCenterPoint.x, lBlockCenterPoint.y + static_cast<int>(BASE_Y_DISTANCE_CM * aCoordinateConversionValue));

          std::cout << "lXCoordinateValue base : " << lBasePoint.x  << std::endl;
          std::cout << "lYCoordinateValue base : " << lBasePoint.y  << std::endl;
          // Draw the base point
          circle(mDisplayImage, lBasePoint, 3, Scalar(0, 0, 255), -1, 8, 0);
          // Draw a line from the basepoint for positioning
          line(mDisplayImage, lBasePoint, Point(0.0, lBasePoint.y), Scalar(0, 255, 0));
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
          imshow("result", mDisplayImage);
        }
      }
    }
    int pressedKey = waitKey(5);
    if (pressedKey == 27) // ESC key
    {
      break;
    }
  }
  // Calculate base coordinate
  double lXCoordinateValue = lBasePoint.x / aCoordinateConversionValue;
  double lYCoordinateValue = lBasePoint.y / aCoordinateConversionValue;
  lReturn.at(0, 0) = lXCoordinateValue;
  lReturn.at(1, 0) = lYCoordinateValue;
  destroyAllWindows();
  return lReturn;
}

double Shapedetector::calibrateCoordinates(int aDeviceId)
{
  const double SHAPE_WIDTH_SIZE_CM = 4.97;
  // Factor to compensate for the camera angle, based on measurements
  const double SHAPE_COMPENSATION_FACTOR = 1.06;
  unsigned int lPixels = 0;
  double lReturn;

  initCamera(aDeviceId);
  Mat firstRetrievedFrame;
  mVidCap.grab();
  mVidCap.retrieve(firstRetrievedFrame);

  // Calibrate the colors used for detecting the shapes
  calibrateColors();

  while (true)
  {
    Mat retrievedFrame;
    mVidCap.grab();
    mVidCap.retrieve(retrievedFrame);
    mOriginalImage = retrievedFrame;
    reset();
    // Recognize calibration object
    // Filter color
    Mat colorMask;
    inRange(mOriginalImage, mRedLimits[0], mRedLimits[1], colorMask);
    imshow("colorMask", colorMask);
    // Find shape
    findContours(colorMask, mCurrentContours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    removeCloseShapes(mCurrentContours);
    for (size_t i = 0; i < mCurrentContours.size(); i++)
    {
      double epsilon = mEpsilonMultiply * arcLength(mCurrentContours.at(i), true);
      approxPolyDP(mCurrentContours.at(i), mApproxImage, epsilon, true);
      if (mApproxImage.size().height == SQUARE_CORNERCOUNT)
      {
        if (contourSizeAllowed(mCurrentContours.at(i), mMinContourSize, mMaxContourSize))
        {
          // Get longest side length
          RotatedRect lRotatedRect = minAreaRect(mCurrentContours.at(i));
          double lMaxDistance = 0.0;
          Point2f vertices[4];
          lRotatedRect.points(vertices);
          for (int j = 0; j < 4; j++)
          {
            line(mDisplayImage, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0));
            double lDistance = (double)cv::norm(vertices[j] - vertices[(j + 1) % 4]);
            if (lDistance > lMaxDistance)
            {
              lMaxDistance = lDistance;
            }
          }
          std::cout << "MaxDistance : " << lMaxDistance << std::endl;
          lPixels = static_cast<unsigned int>(lMaxDistance);
          setShapeValues(mDisplayImage, mCurrentContours.at(i));
          imshow("result", mDisplayImage);
        }
      }
    }
    //  Calculate pixels per centimeter
    lReturn = (double)(lPixels * SHAPE_COMPENSATION_FACTOR) / (double)SHAPE_WIDTH_SIZE_CM;
    std::cout << "Pixels per CM : " << lReturn << std::endl;
    int pressedKey = waitKey(5);
    if (pressedKey == 27) // ESC key
    {
      break;
    }
  }
  destroyAllWindows();
  return lReturn;
}

std::pair<std::vector<Shape>, Matrix<double, 2, 1>> Shapedetector::detectShapeCoordinates(int aDeviceId)
{
  std::pair<std::vector<Shape>, Matrix<double, 2, 1>> lReturnValue;
  Matrix<double, 2, 1> lReturnMatrix;
  initCamera(aDeviceId);
  // Start webcam mode
  std::cout << "### Webcam mode ###" << std::endl;
  std::cout << "Please enter [vorm] [breedte] [hoogte] [kleur]" << std::endl;
  std::cout << "> ";
  std::string command;
  getline(std::cin, command); // Get command

  if (command != EXIT_COMMAND)
  {
    bool parsingSucceeded = parseSpec(command);
    if (parsingSucceeded == false)
    {
      std::cout << "Error: invalid specification entered" << std::endl;
      throw std::invalid_argument("One of the commandline arguments is invalid");
    }
    mShapes.clear();
    detectRealtime();
  }
  lReturnValue.first = mShapes;
  lReturnMatrix.at(0,0) = mFindShapeWidth;
  lReturnMatrix.at(1,0) = mFindShapeHeight;
  lReturnValue.second = lReturnMatrix;
  return lReturnValue;
}

Matrix<double, 2, 1> Shapedetector::detectBaseCoordinates(int deviceId)
{
  initCamera(deviceId);
  Mat firstRetrievedFrame;
  mVidCap.grab();
  mVidCap.retrieve(firstRetrievedFrame);

  while (true)
  {
    Mat retrievedFrame;
    mVidCap.grab();
    mVidCap.retrieve(retrievedFrame);
    mOriginalImage = retrievedFrame;
    reset();
    // Recognize calibration object
    // Create bitmap
    Mat lHSVImage;
    Mat lGreyScale;
    Mat lBitmap;
    Mat lDetectedEdges;
    std::vector<Mat> lChannels;

    cvtColor(mOriginalImage, lHSVImage, CV_BGR2HSV);
    split(lHSVImage, lChannels);

    inRange(lHSVImage, Scalar(0, 0, 220), Scalar(180, 255, 255), lBitmap);
    imshow("circles", lBitmap);

    // Find shape
    findContours(lBitmap, mCurrentContours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    removeCloseShapes(mCurrentContours);
    for (size_t i = 0; i < mCurrentContours.size(); i++)
    {
      if (contourSizeAllowed(mCurrentContours.at(i), mMinBaseContourSize, mMaxBaseContourSize))
      {
        // Get center
        Point circleCenter = getContourCenter(mCurrentContours.at(i));
        drawContours(mOriginalImage, mCurrentContours.at(i), -1, Scalar(0, 255, 0), 3);
        mShapePosition.at(0, 0) = circleCenter.x;
        mShapePosition.at(1, 0) = circleCenter.y;
        circle(mOriginalImage, circleCenter, 3, Scalar(0, 0, 255), -1, 8, 0);
        setShapeValues(mDisplayImage, mCurrentContours.at(i));
      }
    }
    imshow("result", mOriginalImage);
    int pressedKey = waitKey(5);
    if (pressedKey == 27) // ESC key
    {
      break;
    }
  }
  return mShapePosition;
}

void Shapedetector::detectRealtime()
{
  Mat firstRetrievedFrame;
  mVidCap.grab();
  mVidCap.retrieve(firstRetrievedFrame);
  setImage(firstRetrievedFrame);

  draw();

  while (true)
  {
    Mat retrievedFrame;
    mVidCap.grab();
    mVidCap.retrieve(retrievedFrame);
    mOriginalImage = retrievedFrame;
    reset();
    recognize();
    printDetectionData();

    bool keyPressed = showImages();
    if (keyPressed)
    {
      break;
    }
  }
}

void Shapedetector::initCamera(int cameraId)
{
  mVidCap.open(cameraId);

  if (mVidCap.isOpened() == false)
  {
    std::cout << "Error: video capture not opened" << std::endl;
    exit(-1);
  }
}

void Shapedetector::calibrateColors()
{
  // Create sliders
  namedWindow("Color sliders");
  createTrackbar("minHue\t\t", "Color sliders", &mMinCalibrationHue, mCalibrationHueRange, onChange, this);
  createTrackbar("maxHue\t\t", "Color sliders", &mMaxCalibrationHue, mCalibrationHueRange, onChange, this);
  createTrackbar("minSaturation\t\t", "Color sliders", &mMinCalibrationSaturation, mCalibrationSaturationRange, onChange, this);
  createTrackbar("maxSaturation\t\t", "Color sliders", &mMaxCalibrationSaturation, mCalibrationSaturationRange, onChange, this);
  createTrackbar("minValue\t\t", "Color sliders", &mMinCalibrationValue, mCalibrationValueRange, onChange, this);
  createTrackbar("maxValue\t\t", "Color sliders", &mMaxCalibrationValue, mCalibrationValueRange, onChange, this);
  const int sliderWidth = 500;
  Mat emptyMatrix = Mat::zeros(1, sliderWidth, CV_8U);
  imshow("Color sliders", emptyMatrix); // put an empty matrix in this window to prevent errors
  moveWindow("Color sliders", 0, mOriginalImage.rows + 10);

  Mat retrievedFrame;
  Mat maskedFrame;

  COLORS currentColor;
  Scalar minCalibrationValues;
  Scalar maxCalibrationValues;

  // Loop through colors
  for (size_t i = 0; i < COLORSTRINGS.size() - 1; i++)
  {
    // Load saved values
    currentColor = StringToColor(COLORSTRINGS.at(i));
    loadColorValues(currentColor, minCalibrationValues, maxCalibrationValues);
    // Set slider values
    setCurrentSliderValues(minCalibrationValues, maxCalibrationValues);
    // Print color to calibrate
    std::cout << "Calibrating " << COLORSTRINGS.at(i) << " colors." << std::endl;
    while (true) // Escape pressed
    {
      // Capture frame
      if (mVidCap.isOpened())
      {
        mVidCap.grab();
        mVidCap.retrieve(retrievedFrame);
      }

      minCalibrationValues = Scalar(mMinCalibrationHue, mMinCalibrationSaturation, mMinCalibrationValue);
      maxCalibrationValues = Scalar(mMaxCalibrationHue, mMaxCalibrationSaturation, mMaxCalibrationValue);
      inRange(retrievedFrame, minCalibrationValues, maxCalibrationValues, maskedFrame);

      imshow("orgininal", retrievedFrame);
      imshow("mask", maskedFrame);

      int capturedKey = waitKey(30);
      if (capturedKey == 27) // Escape pressed
      {
        break;
      }
    }
    // Save new limits
    saveColorValues(currentColor, minCalibrationValues, maxCalibrationValues);
  }
  destroyAllWindows();
}

void Shapedetector::setCurrentSliderValues(Scalar minCalibrationValues, Scalar maxCalibrationValues)
{
  mMinCalibrationHue = (int)minCalibrationValues[0];
  mMaxCalibrationHue = (int)maxCalibrationValues[0];
  mMinCalibrationSaturation = (int)minCalibrationValues[1];
  mMaxCalibrationSaturation = (int)maxCalibrationValues[1];
  mMinCalibrationValue = (int)minCalibrationValues[2];
  mMaxCalibrationValue = (int)maxCalibrationValues[2];
  cvSetTrackbarPos("minHue\t\t", "Color sliders", mMinCalibrationHue);
  cvSetTrackbarPos("maxHue\t\t", "Color sliders", mMaxCalibrationHue);
  cvSetTrackbarPos("minSaturation\t\t", "Color sliders", mMinCalibrationSaturation);
  cvSetTrackbarPos("maxSaturation\t\t", "Color sliders", mMaxCalibrationSaturation);
  cvSetTrackbarPos("minValue\t\t", "Color sliders", mMinCalibrationValue);
  cvSetTrackbarPos("maxValue\t\t", "Color sliders", mMaxCalibrationValue);
}

void Shapedetector::loadColorValues(COLORS aColor, Scalar &aMinScalar, Scalar &aMaxScalar) const
{
  switch (aColor)
  {
  case (COLORS::RED):
    aMinScalar = mRedLimits[0];
    aMaxScalar = mRedLimits[1];
    break;
  case (COLORS::GREEN):
    aMinScalar = mGreenLimits[0];
    aMaxScalar = mGreenLimits[1];
    break;
  case (COLORS::BLUE):
    aMinScalar = mBlueLimits[0];
    aMaxScalar = mBlueLimits[1];
    break;
  case (COLORS::BLACK):
    aMinScalar = mBlackLimits[0];
    aMaxScalar = mBlackLimits[1];
    break;
  case (COLORS::YELLOW):
    aMinScalar = mYellowLimits[0];
    aMaxScalar = mYellowLimits[1];
    break;
  case (COLORS::WHITE):
    aMinScalar = mWhiteLimits[0];
    aMaxScalar = mWhiteLimits[1];
    break;
  case (COLORS::UNKNOWNCOLOR):
    break;
  }
}

void Shapedetector::saveColorValues(COLORS aColor, Scalar aMinScalar, Scalar aMaxScalar)
{
  switch (aColor)
  {
  case (COLORS::RED):
    mRedLimits[0] = aMinScalar;
    mRedLimits[1] = aMaxScalar;
    break;
  case (COLORS::GREEN):
    mGreenLimits[0] = aMinScalar;
    mGreenLimits[1] = aMaxScalar;
    break;
  case (COLORS::BLUE):
    mBlueLimits[0] = aMinScalar;
    mBlueLimits[1] = aMaxScalar;
    break;
  case (COLORS::BLACK):
    mBlackLimits[0] = aMinScalar;
    mBlackLimits[1] = aMaxScalar;
    break;
  case (COLORS::YELLOW):
    mYellowLimits[0] = aMinScalar;
    mYellowLimits[1] = aMaxScalar;
    break;
  case (COLORS::WHITE):
    mWhiteLimits[0] = aMinScalar;
    mWhiteLimits[1] = aMaxScalar;
    break;
  case (COLORS::UNKNOWNCOLOR):
    break;
  }
}