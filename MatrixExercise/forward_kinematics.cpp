#include <cmath>
#include <tuple>
#include <iostream>

std::tuple<double, double> getEndPos(int beginX, int beginY, int l1, double theta1, int l2, double theta2, int l3, double theta3);

int main(int argc, char** argv)
{
	std::tuple<double, double> curPos = getEndPos(0, 0, 10, 0, 10, 90, 10, 90);
	std::cout << "x: " << std::get<0>(curPos) << " y: " << std::get<1>(curPos) << std::endl;
	return 0;
}

std::tuple<double, double> getEndPos(int beginX, int beginY, int l1, double theta1, int l2, double theta2, int l3, double theta3)
{
	double xPos = (beginX + (l1 * sin(theta1 * (M_PI / 180.0))) + (l2 * sin((theta1 + theta2) * (M_PI / 180.0))) + (l3 * sin((theta1 + theta2 + theta3) * (M_PI / 180.0))));
	double yPos = (beginY + (l1 * cos(theta1 * (M_PI / 180.0))) + (l2 * cos((theta1 + theta2) * (M_PI / 180.0))) + (l3 * cos((theta1 + theta2 + theta3) * (M_PI / 180.0))));
  std::tuple<double, double> resultPos = std::make_tuple(xPos, yPos);
	return resultPos;
}