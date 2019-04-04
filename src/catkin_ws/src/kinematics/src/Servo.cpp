#include "Servo.h"

Servo::Servo(unsigned int aServoId, int aMinDegreesLimit, int aMaxDegreesLimit, int aMinDegreesRange, int aMaxDegreesRange, int aPulsewidthCompensation)
    : mServoId(aServoId), mMinDegreesLimit(aMinDegreesLimit), mMaxDegreesLimit(aMaxDegreesLimit), mMinDegreesRange(aMinDegreesRange), mMaxDegreesRange(aMaxDegreesRange), mCurrentDegrees(0), mPulsewidthCompensation(aPulsewidthCompensation)
{
}

Servo::Servo(const Servo& aOther) : mServoId(aOther.mServoId), mMinDegreesLimit(aOther.mMinDegreesLimit), mMaxDegreesLimit(aOther.mMaxDegreesLimit), mMinDegreesRange(aOther.mMinDegreesRange), mMaxDegreesRange(aOther.mMaxDegreesRange), mCurrentDegrees(aOther.mCurrentDegrees), mPulsewidthCompensation(aOther.mPulsewidthCompensation)
{
}

Servo::~Servo()
{
}

unsigned int Servo::getServoId() const
{
  return mServoId;
}

int Servo::getMinDegreesLimit() const
{
  return mMinDegreesLimit;
}

int Servo::getMaxDegreesLimit() const
{
  return mMaxDegreesLimit;
}

int Servo::getMinDegreesRange() const
{
  return mMinDegreesRange;
}

int Servo::getMaxDegreesRange() const
{
  return mMaxDegreesRange;
}

int Servo::getCurrentDegrees() const
{
  return mCurrentDegrees;
}

int Servo::getPulsewidthCompensation() const
{
  return mPulsewidthCompensation;
}

void Servo::setCurrentDegrees(int aDegrees)
{
  mCurrentDegrees = aDegrees;
}

Servo& Servo::operator=(const Servo& aOther)
{
  if (aOther == *this)
  {
    return *this;
  }
  this->mServoId = aOther.getServoId();
  this->mMinDegreesLimit = aOther.getMinDegreesLimit();
  this->mMaxDegreesLimit = aOther.getMaxDegreesLimit();
  this->mMinDegreesRange = aOther.getMinDegreesRange();
  this->mMaxDegreesRange = aOther.getMaxDegreesRange();
  this->mCurrentDegrees = aOther.getCurrentDegrees();
  this->mPulsewidthCompensation = aOther.getPulsewidthCompensation();
  return *this;
}

bool Servo::operator==(Servo aServo) const
{
  return ((this->mServoId == aServo.mServoId) && (this->mMinDegreesLimit == aServo.mMinDegreesLimit) && (this->mMaxDegreesLimit == aServo.mMaxDegreesLimit) && (this->mMinDegreesRange == aServo.mMinDegreesRange) && (this->mMaxDegreesRange == aServo.mMaxDegreesRange) && (this->mCurrentDegrees == aServo.mCurrentDegrees));
}