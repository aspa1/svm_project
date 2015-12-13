#include "robot_perception.h"

/**
 * @brief Default Constructor. Gets map topic param and laser topic
 * param and subcribes to these topics.
 */
RobotPerception::RobotPerception () 
{	
	ROS_INFO_STREAM("RobotPerception Constructor");
	if(!_n.getParam("/map_topic", _map_topic_param))
	{
		ROS_ERROR("Map topic param does not exist");
	}	
	_map_sub = _n.subscribe(_map_topic_param, 1,
		&RobotPerception::mapCallback, this);

	if(!_n.getParam("/robot_laser_topic", _laser_topic_param))
	{
		ROS_ERROR("Laser topic param does not exist");
	}	
	_laser_sub = _n.subscribe(_laser_topic_param, 1,
		&RobotPerception::laserRangesCallback, this);
}

/**
 * @brief Gets the necessary info for the map (width, height, data)
 * @param occupancy_grid_msg [nav_msgs::OccupancyGrid] Message 
 * containing the map info
 */
void RobotPerception::mapCallback (
	nav_msgs::OccupancyGrid occupancy_grid_msg)
{		
	_map_width = occupancy_grid_msg.info.width;	
	_map_height = occupancy_grid_msg.info.height;
	
	ROS_INFO_STREAM ("RobotPerception:map_width ="<< " " << _map_width << 
		" " << "RobotPerception:map_height ="<< " " << _map_height);

	_map_data = new int*[_map_width];
	for (unsigned int i = 0 ; i < _map_width ; i++)
	{
		_map_data[i] = new int[_map_height];
	}

	for (unsigned int j = 0 ; j < _map_height ; j++)
	{
		for (unsigned int i = 0 ; i < _map_width ; i++)
		{
			_map_data[i][j] =
				(int)occupancy_grid_msg.data[_map_width*j + i];
		}
	}	
	
}

/**
 * @brief Gets the laser ranges
 * @param laser_scan_msg [sensor_msgs::LaserScan] Message 
 * containing the laser ranges info
 */
void RobotPerception::laserRangesCallback(
	sensor_msgs::LaserScan laser_scan_msg) 
{
	_laser_ranges = laser_scan_msg.ranges;
}

/**
 * @brief Returns the map width
 * @return unsigned int - Map width
 */
unsigned int RobotPerception::getMapWidth()
{
	return _map_width;
}

/**
 * @brief Returns the map height
 * @return unsigned int - Map height
 */	
unsigned int RobotPerception::getMapHeight()
{
	return _map_height;
}

/**
 * @brief Returns the occupancy state of a map cell
 * @param i [int] Coordinate x of the map data array 
 * @param j [int] Coordinate y of the map data array 
 * @return int - Map cell occupancy state 
 */
int RobotPerception::getMapCell(int i, int j) 
{
	return _map_data[i][j];
}

/**
 * @brief Returns the map data
 * @return int** - Map occupancy data array
 */
int** RobotPerception::getMapData ( ) 
{
	return _map_data;
}
/**
 * @brief Returns the laser ranges
 * @return float* - Laser ranges array
 */
std::vector<float> RobotPerception::getLaserRanges() 
{
	return _laser_ranges;
}
