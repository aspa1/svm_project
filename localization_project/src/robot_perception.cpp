#include "robot_perception.h"

RobotPerception::RobotPerception () 
{
	if(!_n.getParam("/map_topic", _map_topic_param))
	{
		ROS_ERROR("Map topic param does not exist");
	}	
	_map_sub = _n.subscribe(_map_topic_param, 5, &RobotPerception::mapCallback, this);

	if(!_n.getParam("/robot_laser_topic", _laser_topic_param))
	{
		ROS_ERROR("Laser topic param does not exist");
	}	
	_laser_sub = _n.subscribe(_laser_topic_param, 5, &RobotPerception::laserRangesCallback, this);
}

void RobotPerception::mapCallback(nav_msgs::OccupancyGrid msg) {
	_map = msg;	
}

void RobotPerception::laserRangesCallback(sensor_msgs::LaserScan msg) {
	_laser_ranges = msg.ranges;
	for (int i=0; i<_laser_ranges.size(); i++) {
		ROS_INFO_STREAM(_laser_ranges[i]);
	}
}


int main (int argc, char **argv) {
	
	ros::init(argc, argv, "main");
	RobotPerception a;
	ros::spin(); 
	return 0;
}
