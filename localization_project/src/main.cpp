#include "robot_perception.h"

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "main");
	RobotPerception a;
	ros::spin(); 
	return 0;
}
