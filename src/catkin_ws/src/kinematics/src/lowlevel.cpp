#include "kinematics/lowlevel.hpp"

lowlevel::lowlevel() : serial(ioservice), mArmLocked(false)
{
  // Set the servo ranges
  mServos.push_back(Servo(0, -100, 100, -100, 100));
  mServos.push_back(Servo(1, -30, 90, -90, 90));
  mServos.push_back(Servo(2, 0, 135, 0, 180));
  mServos.push_back(Servo(3, -90, 90, -90, 90));
  mServos.push_back(Servo(4, 0, 180, 0, 180));
  mServos.push_back(Servo(5, -90, 90, -90, 90));

  // Initializing serial
  boost::system::error_code ec;

  serial.open("/dev/ttyUSB0", ec);

  if (!ec)
  {
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
  }
}

lowlevel::~lowlevel()
{
}

bool lowlevel::moveServosToPos(std::vector<unsigned int> aPins, std::vector<int> aDegrees, unsigned int aMillis)
{
  // If arm is locked, any movement commands shud be ignored
  if(mArmLocked)
  {
    ROS_WARN("Arm is locked, refusing to send movement commands to it");
    return false;
  }

  // Every pin should correspond with a degree value, if this is not the case do nothing (input invalid)
  if (!(aPins.size() == aDegrees.size()))
  {
    std::cout << "Error, moveServosToPos invalid command received: Amount of pins does not match amount of degrees " << std::endl;
    return false;
  }
  // Check if given pins exist
  for (int servoId : aPins)
  {
    if (!servoExists(servoId))
    {
      std::cout << "Given pins in moveServosToPos are not valid, ignoring command" << std::endl;
      return false;
    }
  }
  // Check if given degrees are within the min/max of the servos
  for (int i = 0; i < aDegrees.size(); ++i)
  {
    if (!degreesInRange(aDegrees.at(i), getServoWithId(aPins.at(i))))
    {
      std::cout << "Degree: " << std::to_string(aDegrees.at(i)) << " and servomin: " << std::to_string(getServoWithId(aPins.at(i)).getMinDegreesLimit()) << " and servomax: " << std::to_string(getServoWithId(aPins.at(i)).getMaxDegreesLimit()) << std::endl;
      std::cout << "Not all of the given degrees in moveServosToPos are within the range of the corresponding servos, ignoring command" << std::endl;
      if(aDegrees.at(i) < getServoWithId(aPins.at(i)).getMinDegreesLimit())
      {
        aDegrees.at(i) = getServoWithId(aPins.at(i)).getMinDegreesLimit();
      }
      else if(aDegrees.at(i) > getServoWithId(aPins.at(i)).getMaxDegreesLimit())
      {
        aDegrees.at(i) = getServoWithId(aPins.at(i)).getMaxDegreesLimit();
      }
      // return false;
    }
  }

  std::string lCommand = "";

  for (int i = 0; i < aPins.size(); ++i)
  {
    unsigned int lPulseWidth = convertDegreesToPulsewidth(aDegrees.at(i), getServoWithId(aPins.at(i)));

    lCommand.append("#" + std::to_string(aPins.at(i)));
    lCommand.append("P" + std::to_string(lPulseWidth));
  }

  lCommand.append("T" + std::to_string(aMillis));
  lCommand.append("\r");

  sendSerial(lCommand);
  std::cout << "Sending command : " << lCommand << std::endl;
  return true;
}

unsigned int lowlevel::convertDegreesToPulsewidth(int aDegrees, Servo& aServo) const
{
  unsigned int lPulseRange = MAX_PULSEWIDTH - MIN_PULSEWIDTH;

  unsigned int lMappedValue = mapValues(aDegrees, aServo.getMinDegreesRange(), aServo.getMaxDegreesRange(), 0, std::abs(aServo.getMaxDegreesRange() - aServo.getMinDegreesRange()));

  std::cout << "MappedValue : " << lMappedValue << std::endl;

  unsigned int lDegreeRange = static_cast<unsigned int>(std::abs(aServo.getMaxDegreesRange() - aServo.getMinDegreesRange()));

  std::cout << "lDegreeRange : " << lDegreeRange << std::endl;

  double lFactor = static_cast<double>(lMappedValue) / static_cast<double>(lDegreeRange);

  std::cout << "lFactor : " << lFactor << std::endl;
  
  unsigned int lPulsewidthCompensation = 0;
  if(aServo.getServoId() == 0)
  {
    lPulsewidthCompensation = -50;
  }
  else if(aServo.getServoId() == 1)
  {
    lPulsewidthCompensation = 30;
  }
  else if(aServo.getServoId() == 2)
  {
    lPulsewidthCompensation = 180;
  }
  unsigned int lReturn = static_cast<unsigned int>((MIN_PULSEWIDTH + (lPulseRange * lFactor)) + lPulsewidthCompensation);

  return lReturn;
}

void lowlevel::stopServos(std::vector<unsigned int> aPins)
{
  if (aPins.size() > 0)
  {
    std::string lCommand = "";

    for (int i = 0; i < aPins.size(); ++i)
    {
      lCommand.append("STOP" + std::to_string(aPins.at(i)));
      lCommand.append("\r");
    }
    sendSerial(lCommand);
  }
}

bool lowlevel::degreesInRange(int aDegrees, Servo& aServo) const
{
  bool lReturn = false;

  if ((aDegrees >= aServo.getMinDegreesLimit()) && (aDegrees <= aServo.getMaxDegreesLimit()))
  {
    lReturn = true;
  }

  return lReturn;
}

void lowlevel::sendSerial(std::string aCommand)
{
  boost::asio::streambuf b;
  boost::system::error_code ec;

  std::ostream os(&b);
  os << aCommand << std::endl;

  if (serial.is_open())
  {
    boost::asio::write(serial, b.data());
    os.flush();
  }
  else
  {
    std::cout << "Unable to open serial, stopping program" << std::endl;
    exit(-1);
  }
}

void lowlevel::setBaudRate(unsigned int aBaudRate)
{
  boost::system::error_code ec;
  if (!ec)
  {
    serial.set_option(boost::asio::serial_port_base::baud_rate(aBaudRate));
  }
}

bool lowlevel::servoExists(unsigned int aServoId) const
{
  bool lReturn = false;

  for (int i = 0; (i < mServos.size()) && (!lReturn); ++i)
  {
    if (mServos.at(i).getServoId() == aServoId)
    {
      lReturn = true;
    }
  }

  return lReturn;
}

Servo& lowlevel::getServoWithId(unsigned int aServoId)
{
  bool lFoundServo = false;

  for (int i = 0; i < mServos.size(); ++i)
  {
    if (mServos.at(i).getServoId() == aServoId)
    {
      lFoundServo = true;
      return mServos.at(i);
    }
  }

  if (!lFoundServo)
  {
    throw std::invalid_argument("Invalid servoId entered.");
  }
}

void lowlevel::setArmLocked(bool aLocked)
{
  mArmLocked = aLocked;
}

bool lowlevel::isArmLocked() const
{
  return mArmLocked;
}

unsigned int lowlevel::checkTimeToMoveInRange(std::vector<unsigned int> aPins, std::vector<int> aDegrees, unsigned int aMillis)
{
  std::vector<unsigned int> lMovementRanges;
  for (size_t i = 0; i < aPins.size(); i++)
  {
    Servo lServo = getServoWithId(aPins.at(i));
    // Calculate the difference between the current degree and the goal degree
    int lCurrentDegreeDifference = abs(lServo.getCurrentDegrees() - aDegrees.at(lServo.getServoId()));
    lMovementRanges.push_back(lCurrentDegreeDifference);
  }
  // Get the biggest change
  double lMaxChange = static_cast<double>(*std::max_element(lMovementRanges.begin(), lMovementRanges.end()));

  // Calculate time needed for that movement
  double lTimeNeededToCompleteMove = lMaxChange * MS_PER_DEGREE;

  // Check if the time (aMillis) is within the time actually needed to complete the movements:
  // If not the case, warn and return timeframe needed
  if (static_cast<double>(aMillis) < lTimeNeededToCompleteMove)
  {
    ROS_WARN("QoS-Warning: {Move has goal time of %d ms to be completed, but move needs %d ms}", aMillis, static_cast<unsigned int>(lTimeNeededToCompleteMove));
    return static_cast<unsigned int>(lTimeNeededToCompleteMove);
  }
  else // If move can be handled within the aMillis, return this original timeframe.
  {
    return aMillis;
  }
}

unsigned int lowlevel::mapValues(int aDegree, int aInMin, int aInMax, int aOutMin, int aOutMax) const
{
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}
