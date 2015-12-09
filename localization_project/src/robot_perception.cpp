#include "robot_perception.h"

RobotPerception::RobotPerception () //Constructor
{	
	//Fetching the map topic URI param
	if(!_n.getParam("/map_topic", _map_topic_param))
	{
		ROS_ERROR("Map topic param does not exist");
	}	
	//Subscribing to the map topic
	_map_sub = _n.subscribe(_map_topic_param, 5, &RobotPerception::mapCallback, this);

	//Fetching the laser topic URI param
	if(!_n.getParam("/robot_laser_topic", _laser_topic_param))
	{
		ROS_ERROR("Laser topic param does not exist");
	}	
	//Subscribing to the laser topic
	_laser_sub = _n.subscribe(_laser_topic_param, 5, &RobotPerception::laserRangesCallback, this);
}

void RobotPerception::mapCallback(nav_msgs::OccupancyGrid occupancy_grid_msg) //Function to get the data from the map
{
	//Getting the width & height of the map
	_map_width = occupancy_grid_msg.info.width;	
	_map_height = occupancy_grid_msg.info.height;
	
	//Memory allocation
	_map_data = new int*[_map_width];
	for (unsigned int i = 0 ; i < _map_width ; i++)
	{
		_map_data[i] = new int[_map_height];
	}

	//Copying the data of OccupancyGrid message into _map_data
	for (unsigned int j = 0 ; j < _map_height ; j++)
	{
		for (unsigned int i = 0 ; i < _map_width ; i++)
		{
			_map_data[i][j] = (int)occupancy_grid_msg.data[_map_width*j + i];
		}
	}	
	
	//Memory deallocation
	for (unsigned int i = 0 ; i < _map_width ; i++ )
    {
		delete [] _map_data[i];
	}
	delete [] _map_data;	
}

void RobotPerception::laserRangesCallback(sensor_msgs::LaserScan laser_scan_msg) //Function to get the laser's ranges
{
	//Memory allocation
	_laser_ranges = new float[laser_scan_msg.ranges.size()];
	
	//Copying the data of LaserScan message into _laser_ranges
	for (unsigned int i = 0 ; i < laser_scan_msg.ranges.size() ; i++ ) 
	{
		_laser_ranges[i] = laser_scan_msg.ranges[i];
	}
	
	//Memory deallocation
	delete [] _laser_ranges;
}

int** RobotPerception::getMapData () //Function to get the private class member _map_data
{
	return _map_data;
}

float* RobotPerception::getLaserRanges() //Function to get the private class member _laser_ranges
{
	return _laser_ranges;
}
