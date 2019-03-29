/**
 * @file Servo.h
 * @author Dibran & Marnix
 * @brief Servo
 * @version 1.0
 * @date 2019-02-18
 * @copyright Copyright (c) 2019
 */

#ifndef SERVO_H_
#define SERVO_H_

class Servo
{
  public:
    /**
     * @brief Construct a new Servo object
     * 
     * @param aServoId - Id of the servo, this should correspond with the Id/pin of the real servo
     * @param aMinDegreesLimit - Minimum amount of degrees required to be considered a "safe/allowed" amount of degrees, software constraint.
     * @param aMaxDegreesLimit - Maximum amount of degrees required to be considered a "safe/allowed" amount of degrees, software constraint.
     * @param aMinDegreesRange - Lower limit of the range of degrees the servo is actually able to move (hardware-wise).
     * @param aMaxDegreesRange - Uppr limit of the range of degrees the servo is actually able to move (hardware-wise).
     */
    Servo(unsigned int aServoId, int aMinDegreesLimit, int aMaxDegreesLimit, int aMinDegreesRange, int aMaxDegreesRange);
    
    /**
     * @brief Copy constructor
     * @param aOther - The servo object to be copied.
     */
    Servo(const Servo& aOther); 
    
    /**
     * @brief Destroy the Servo object
     */
    virtual ~Servo();

    // Forbidden default constructor, no point in having a servo object without the right attribute values.
    Servo() = delete;

    /**
     * @brief Get the servoId
     * @return The servoId
     */
    unsigned int getServoId() const;

    /**
     * @brief Get the min degree limit
     * @return The lower bound of the allowed range
     */
    int getMinDegreesLimit() const;

    /**
     * @brief Get the max degrees limit
     * @return The upper bound of the allowed range
     */
    int getMaxDegreesLimit() const;

    /**
     * @brief Get the min degrees range
     * @return The lower bound of the total range supported by hardware
     */
    int getMinDegreesRange() const;

    /**
     * @brief Get the max degrees range
     * @return The upper bound of the total range supported by hardware
     */
    int getMaxDegreesRange() const;

    /**
     * @brief Get the current degrees, that is the amount of degrees which was last sent as a command to the servo.
     * @return The amount of degrees
     */
    int getCurrentDegrees() const;

    /**
     * @brief Sets the current degrees, is usually called when a move instruction is sent via serial to the servo.
     */
    void setCurrentDegrees(int aDegrees);

    Servo& operator=(Servo aOther); // Assignment operator
    bool operator==(Servo aServo); // Comparison operator

  private:
    unsigned int mServoId;

    // Min/Max degrees limit are the limits defined in software here
    int mMinDegreesLimit;
    int mMaxDegreesLimit;

    // Min/Max degrees range are the limits of the range supported by the hardware
    int mMinDegreesRange;
    int mMaxDegreesRange;
    
    // Current degrees of the servo, to be more precise: The number of degrees the servo lastly got instructed to move to.
    int mCurrentDegrees;
};

#endif