#include "RobotarmController.h"

#include <iostream>

int main(int argc,
         char **argv)
{
  if (argc >= 2)
  {


      
    RobotarmController lRobotarmController(atoi(argv[1]));
    lRobotarmController.run();
  }
  else
  {
    std::cout << "Invalid usage, expected form ./kinematics [webcamId]" << std::endl;
  }

  return 0;
}
