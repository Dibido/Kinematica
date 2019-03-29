#include "highlevel.h"
#include "kinematics/lowlevel.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "highlevel");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO("Starting Servo high level driver");
  highlevel lHighlevelDriver;
  if (argc == 1)
  {
    // Default baudrate, if none is given by user
    lHighlevelDriver.setBaudRate(DEFAULT_BAUDRATE);
  }
  else if (argc == 2)
  {
    lHighlevelDriver.setBaudRate(atoi(argv[1]));
  }
  else
  {
    std::cout << "Usage : higleveldriver [baudrate]" << std::endl;
    exit(0);
  }

  lHighlevelDriver.run();

  return 0;
}

highlevel::highlevel() : mMoveCommandSent(false), mIsReady(false), mIsInitialized(false), mInitializeTime(INITIALIZE_TIME), mBaudRate(DEFAULT_BAUDRATE), mLastMoveTimeMS(0), mLastMoveStartTime(std::chrono::high_resolution_clock::now())
{
  subscribeTopics();
  initializeValues();
  initializeArm();
}

highlevel::highlevel(unsigned int aBaudRate) : mMoveCommandSent(false), mIsReady(false), mIsInitialized(false), mInitializeTime(INITIALIZE_TIME), mBaudRate(DEFAULT_BAUDRATE), mLastMoveTimeMS(0), mLastMoveStartTime(std::chrono::high_resolution_clock::now())
{
  subscribeTopics();
  initializeValues();
  initializeArm();
}

highlevel::~highlevel()
{
}

void highlevel::setBaudRate(unsigned int aBaudRate)
{
  mLowLevelDriver.setBaudRate(aBaudRate);
}

void highlevel::run()
{
  while (true)
  {
    ros::spinOnce();
    handleMoveCommands();
  }
}

void highlevel::signalHandler(int aSignal)
{
  // Used when Ctrl-C pressed, shut down
  exit(1);
}

void highlevel::singleServoCallback(const robotarminterface::singleServoConstPtr& aSingleServoMessage)
{
  ROS_INFO("Handling command, Position %d, Time %d", aSingleServoMessage->position, aSingleServoMessage->time);
  std::vector<unsigned int> lPins;
  lPins.push_back(aSingleServoMessage->servoId);
  std::vector<int> lDegrees;
  lDegrees.push_back(aSingleServoMessage->position);
  unsigned int lMillis = aSingleServoMessage->time;
  mLowLevelDriver.moveServosToPos(lPins, lDegrees, lMillis);
}

void highlevel::stopSingleServoCallback(const robotarminterface::stopSingleServoConstPtr& aStopSingleServoMessage)
{
  ROS_INFO("Handling stop command, Id %d", aStopSingleServoMessage->servoId);
  std::vector<unsigned int> lPins;
  lPins.push_back(aStopSingleServoMessage->servoId);
  mLowLevelDriver.stopServos(lPins);
}

void highlevel::moveServosCallback(const robotarminterface::moveServosConstPtr& aMoveServosMessage)
{
  ROS_INFO("Receiving moveMultipleServos command");

  robotarmPosition lMoveCommand;

  for (int i = 0; i < aMoveServosMessage->servos.size(); ++i)
  {
    lMoveCommand.servoIds.push_back(aMoveServosMessage->servos[i].servoId);
    lMoveCommand.servoDegrees.push_back(aMoveServosMessage->servos[i].position);
    std::cout << "Servo : " << aMoveServosMessage->servos[i].servoId << " Angle : " << aMoveServosMessage->servos[i].position << std::endl;
  }

  lMoveCommand.time = aMoveServosMessage->time;

  mMoveCommands.push_back(lMoveCommand);
}

void highlevel::stopServosCallback(const robotarminterface::stopServosConstPtr& aStopServosMessage)
{
  ROS_INFO("Handling stopServos Command");
  mLowLevelDriver.stopServos(aStopServosMessage->servoIds);

  mMoveCommands.clear();
}

void highlevel::armInstructionCallback(const robotarminterface::armInstructionConstPtr& aArmInstructionMessage)
{
  ROS_INFO("Receiving armPosition command, position : %s, time : %d", aArmInstructionMessage->instruction.c_str(), aArmInstructionMessage->time);

  bool lValidCommand = false;
  robotarmPosition lMoveCommand;

  if (aArmInstructionMessage->instruction == "park")
  {
    lMoveCommand.servoIds = mParkPosition.servoIds;
    lMoveCommand.servoDegrees = mParkPosition.servoDegrees;
    lMoveCommand.time = aArmInstructionMessage->time;
    mMoveCommands.push_back(lMoveCommand);
  }
  else if (aArmInstructionMessage->instruction == "ready")
  {
    lMoveCommand.servoIds = mReadyPosition.servoIds;
    lMoveCommand.servoDegrees = mReadyPosition.servoDegrees;
    lMoveCommand.time = aArmInstructionMessage->time;
    mMoveCommands.push_back(lMoveCommand);
  }
  else if (aArmInstructionMessage->instruction == "straight")
  {
    lMoveCommand.servoIds = mStraightPosition.servoIds;
    lMoveCommand.servoDegrees = mStraightPosition.servoDegrees;
    lMoveCommand.time = aArmInstructionMessage->time;
    mMoveCommands.push_back(lMoveCommand);
  }
  else if (aArmInstructionMessage->instruction == "stop")
  {
    ROS_DEBUG("EVENT: {stop}");
    ROS_INFO("STATE: {stopped}");
    mLowLevelDriver.stopServos(mServoIds);
    mMoveCommands.clear();
    mLowLevelDriver.setArmLocked(true);
  }
  else if (aArmInstructionMessage->instruction == "release")
  {
    ROS_DEBUG("EVENT: {release}");
    mLowLevelDriver.setArmLocked(false);
    mMoveCommands.clear();
  }
  else if (aArmInstructionMessage->instruction == "shutdown")
  {
    ROS_DEBUG("EVENT: {shutdown}");
    mMoveCommands.clear();
    mLowLevelDriver.moveServosToPos(mParkPosition.servoIds, mParkPosition.servoDegrees, mInitializeTime);
    exit(-1);
  }
}

void highlevel::initializeArm()
{
  ROS_INFO("STATE: {initializing}");

  // Go to park
  robotarmPosition lMoveCommand;

  lMoveCommand.servoIds = mParkPosition.servoIds;
  lMoveCommand.servoDegrees = mParkPosition.servoDegrees;
  lMoveCommand.time = mInitializeTime;
  mMoveCommands.push_back(lMoveCommand);
}

void highlevel::subscribeTopics()
{
  mSingleServoSubscriber = mNodeHandler.subscribe("singleServo", 1000, &highlevel::singleServoCallback, this);
  mStopSingleServoSubscriber = mNodeHandler.subscribe("stopSingleServo", 1000, &highlevel::stopSingleServoCallback, this);
  mMoveServosSubscriber = mNodeHandler.subscribe("moveServos", 1000, &highlevel::moveServosCallback, this);
  mStopServosSubscriber = mNodeHandler.subscribe("stopServos", 1000, &highlevel::stopServosCallback, this);
  mArmInstructionSubscriber = mNodeHandler.subscribe("armInstructionPosition", 1000, &highlevel::armInstructionCallback, this);
}

void highlevel::initializeValues()
{
  // Used to catch ctrl-c event
  signal(SIGINT, signalHandler);

  mServoIds = {0, 1, 2, 3, 4, 5};

  // Understanding values have been found by experimenting with the robot arm, and may differ slightly from robot to robot.
  mParkPosition.servoIds = {0, 1, 2, 3, 4, 5};
  mParkPosition.servoDegrees = {0, 30, 135, -60, 180, 0};
  mReadyPosition.servoIds = {0, 1, 2, 3, 4, 5};
  mReadyPosition.servoDegrees = {0, 30, 120, 0, 0, 0};
  mStraightPosition.servoIds = {0, 1, 2, 3, 4, 5};
  mStraightPosition.servoDegrees = {0, 0, 30, 20, 0, 0};

  mLowLevelDriver.setBaudRate(mBaudRate);
}

void highlevel::handleMoveCommands()
{
  /* If not initialized before, but there was a (initialize/park)move command sent and 
  the time needed to complete the command has expired */
  if (!mIsInitialized && mMoveCommandSent && lastMovePeriodExpired())
  {
    mIsInitialized = true;
    mIsReady = true;
    ROS_INFO("STATE: {ready}");
  }

  /* If initialization has been done, and there are no new commands. 
  Robot was moving and time needed to complete last move has elapsed.
  Arm also not locked */
  if (mMoveCommands.size() == 0 && lastMovePeriodExpired() && !mIsReady && mIsInitialized && !mLowLevelDriver.isArmLocked())
  {
    mIsReady = true;
    ROS_INFO("STATE: {ready}");
  }

  // Last move had time to be excuted and there are remainig move commands in queue:
  if (lastMovePeriodExpired() && mMoveCommands.size() > 0)
  {
    // Select first command
    robotarmPosition lNewMoveCommand = mMoveCommands.at(0);

    // Check if move can be completed within the given time, if this is not the case time will get corrected.
    lNewMoveCommand.time = mLowLevelDriver.checkTimeToMoveInRange(lNewMoveCommand.servoIds, lNewMoveCommand.servoDegrees, lNewMoveCommand.time);
    
    // Send move command to lowlevel
    bool lValidMove = mLowLevelDriver.moveServosToPos(lNewMoveCommand.servoIds, lNewMoveCommand.servoDegrees, lNewMoveCommand.time);

    // Remove the first command of the list
    mMoveCommands.erase(mMoveCommands.begin());

    // If the command was valid and thus sent through to the robot
    if (lValidMove)
    {
      // Set MoveCommand, so it is known atleast one valid move command has been sent by serial
      if (!mMoveCommandSent)
      {
        mMoveCommandSent = true;
      }

      if (mIsReady)
      {
        // Robot will move, so we're not ready
        mIsReady = false;
        ROS_DEBUG("EVENT: {handleCommand}");
        ROS_INFO("STATE: {moving}");
      }

      // Update times
      mLastMoveStartTime = std::chrono::high_resolution_clock::now();
      mLastMoveTimeMS = lNewMoveCommand.time;
    }
  }
}

bool highlevel::lastMovePeriodExpired() const
{
  auto lTime = std::chrono::high_resolution_clock::now();

  if (std::chrono::duration_cast<std::chrono::milliseconds>(lTime - mLastMoveStartTime).count() > mLastMoveTimeMS)
  {
    return true;
  }

  return false;
}