#ifndef ROBOT_PERCEPTION_H
#define ROBOT_PERCEPTION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class RobotPerception
{
	private:
		ros::NodeHandle _n;
		std::string _map_topic_param;
		std::string _laser_topic_param;
		ros::Subscriber _map_sub;
		ros::Subscriber _laser_sub;
		unsigned int _map_width;
		unsigned int _map_height;
		float _map_resolution;
		int** _map_data; 
		std::vector<float> _laser_ranges;
	
	
	public:
		RobotPerception();
		void mapCallback(nav_msgs::OccupancyGrid occupancy_grid_msg);
		void laserRangesCallback(sensor_msgs::LaserScan laser_scan_msg);
		unsigned int getMapWidth();
		unsigned int getMapHeight();
		float getMapResolution();
		int** getMapData();
		int getMapCell (int i, int j);
		std::vector<float> getLaserRanges();
};

#endif
