#include "robot_perception.h"

/**
 * @brief Default Constructor. Gets map topic param and laser topic
 * param and subcribes to these topics.
 */
RobotPerception::RobotPerception () 
{
	_qr_file_path = ros::package::getPath("localization_project") + "/cfg/AllQRs.txt";

	_flag = false;

	_rfid_tags_visualization_service = _n.advertiseService (
		"rfid_tags_visualization", &RobotPerception::rfidTagsVisualize, this);
	ROS_INFO("Service ready to visualize RFIDs");

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
		
	_visualization_pub = _n.advertise<visualization_msgs::Marker>(
		"visualization_marker", 0);
	
	if (!_n.getParam("/stdr_used", _stdr_in_use))
	{
		ROS_ERROR("Stdr_used param does not exist");
	}
	
	if (_stdr_in_use)
	{
		if(!_n.getParam("/robot_param", _robot_param))
		{
			ROS_ERROR("Robot param does not exist");
		}
	
		if(!_n.getParam("/laser_param", _laser_param))
		{
			ROS_ERROR("Laser param does not exist");
		}
	}
	
	if (!_n.getParam("/rfid_reader_exists", _rfid_reader_exists))
	{
		ROS_ERROR("Rfid_reader_exists param does not exist");
	}

	if (_rfid_reader_exists)
	{
		if (_stdr_in_use) 
		{
			if (!_n.getParam("/rfid_tags_topic", _rfid_tags_topic_param))
			{
				ROS_ERROR("Rfid_tags topic param does not exist");
			}

			_rfid_tags_sub = _n.subscribe(_rfid_tags_topic_param, 1,
				&RobotPerception::rfidTagsCallback, this);
		}
		else
		{
			rfidFileRead();
		}
			
		if (!_n.getParam("/rfid_reader_topic", _rfid_reader_topic_param))
		{
			ROS_ERROR("Rfid_reader topic param does not exist");
		}
		_rfid_reader_sub = _n.subscribe(_rfid_reader_topic_param, 2,
			&RobotPerception::rfidReaderCallback, this);
	}
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
	_map_resolution = occupancy_grid_msg.info.resolution; 
	
	//~ ROS_INFO_STREAM ("RobotPerception:map_width ="<< " " << _map_width << 
		//~ " " << "RobotPerception:map_height ="<< " " << _map_height);

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

void RobotPerception::rfidTagsCallback (stdr_msgs::RfidTagVector
	rfid_tag_msg)
{
	_rfid_tags_id.clear();
	_rfid_tags_x.clear();
	_rfid_tags_y.clear();
	//~ ROS_ERROR_STREAM("rfidTagsCallback");
	_rfid_tags = rfid_tag_msg.rfid_tags;
	 for (unsigned int i = 0 ; i < _rfid_tags.size(); i++)
	{
		std::size_t found = _rfid_tags[i].tag_id.find("Loc");
		if (found!=std::string::npos)
		{
			_rfid_tags_id.push_back(_rfid_tags[i].tag_id);
			_rfid_tags_x.push_back(_rfid_tags[i].pose.x);
			_rfid_tags_y.push_back(_rfid_tags[i].pose.y);
		}
	}
	//~ std::ofstream data_file;
	//~ data_file.open(_qr_file_path.c_str());
	//~ for (unsigned int i = 0 ; i < _rfid_tags.size(); i++)
	//~ {
		//~ data_file << _rfid_tags[i].tag_id << "\t" <<
			//~ _rfid_tags[i].pose.x << "\t" << _rfid_tags[i].pose.y << "\n"; 
	//~ }
	//~ data_file.close();
	//~ 
	//~ rfidFileRead();
}

void RobotPerception::rfidFileRead()
{
	std::string line;
	std::ifstream qr_file (_qr_file_path.c_str());
	if (qr_file.is_open())
	{
		while (getline (qr_file,line))
		{
			std::string id;
			float x, y;
			std::istringstream ss(line);
			ss >> id >> x >> y;
			std::size_t found = id.find("Loc");
			if (found!=std::string::npos)
			{
				_rfid_tags_id.push_back(id);
				_rfid_tags_x.push_back(x);
				_rfid_tags_y.push_back(y);
			}
		}
	}
	qr_file.close();
}

bool RobotPerception::rfidTagsVisualize (
	localization_project::rfidTagsVisualizeSrv::Request& req,
	localization_project::rfidTagsVisualizeSrv::Response& res
	)
{	
	int counter = 0;
	for(unsigned int i = 0 ; i < _rfid_tags_id.size() ; i++)
	{
		std::string id = _rfid_tags_id[i];
		float x = _rfid_tags_x[i];
		float y = _rfid_tags_y[i];
		visualization_msgs::Marker m, m1;

		//~ ROS_INFO_STREAM("Localization QR");
		//~ ROS_INFO_STREAM("_rfid_tags_id = " << id.c_str());
						
		m.header.frame_id = "map";
		m.header.stamp = ros::Time();
		m.type = visualization_msgs::Marker::SPHERE_LIST;
		m.action = visualization_msgs::Marker::ADD;
		m.id = counter;
		m.ns = "Localization_QRs";
		m.scale.x = 0.35;
		m.scale.y = 0.35;
		m.scale.z = 0.35;
		m.color.a = 1.0;
		m.color.r = 1.0;
		m.color.g = 0.0;
		m.color.b = 1.0;
					
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		m.points.push_back(p);
		_visualization_pub.publish(m);
		//~ ROS_INFO_STREAM("Visualization marker published");
		
		m1.header.frame_id = "map";
		m1.header.stamp = ros::Time();
		m1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		m1.action = visualization_msgs::Marker::ADD;
		m1.id = counter;
		m1.ns = "Localization_QR_id";
		m1.scale.z = 0.5;
		m1.color.a = 1.0;
		m1.color.r = 0.0;
		m1.color.g = 1.0;
		m1.color.b = 0.0;
		m1.text = id;
		//~ ROS_INFO_STREAM("Description = " << m1.text);
		m1.pose.position.x = x;
		m1.pose.position.y = y;
		geometry_msgs::Point p1;
		p1.x = x;
		p1.y = y;
		m1.points.push_back(p1);
		
		_visualization_pub.publish(m1);
		//~ ROS_INFO_STREAM("x = " << x  << " y = " << y );
		counter++;
	}
	
	//~ ROS_INFO_STREAM(counter << " " << "Localization QRs visualized");
	
	res.success = true;
	return true;
}

void RobotPerception::rfidReaderCallback (stdr_msgs::RfidSensorMeasurementMsg
	rfid_reader_msg)
{
	_rfid_pose.clear();
	_rfid_ids = rfid_reader_msg.rfid_tags_ids;

	//~ std::vector<std::vector<float> > tmp_rfids;
	for (unsigned int i = 0 ; i < _rfid_ids.size() ; i++)
	{
		for (unsigned int j = 0 ; j < _rfid_tags_id.size() ; j++)
		{
			if (!_rfid_ids[i].compare(_rfid_tags_id[j]))
			{
				std::vector<float> temp;
				temp.push_back(_rfid_tags_x[j]);
				temp.push_back(_rfid_tags_y[j]);
				_rfid_pose.push_back(temp);
				//~ tmp_rfids.push_back(temp);
			}
		}
	}
	//~ _rfid_pose = tmp_rfids;
}

/**
 * @brief Gets the laser ranges
 * @param laser_scan_msg [sensor_msgs::LaserScan] Message 
 * containing the laser ranges info
 */
void RobotPerception::laserRangesCallback(
	sensor_msgs::LaserScan laser_scan_msg) 
{
	if (_stdr_in_use)
	{
		tf::StampedTransform transform;
		if (!_flag)
		{
			try
			{
				_listener.lookupTransform(_robot_param, _laser_param, ros::Time(0), transform);
				yy = tf::getYaw(transform.getRotation());
				ROS_ERROR_STREAM(yy);	
				_flag = true;
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}
	}
	else
		yy = 0;
	_increment = laser_scan_msg.angle_increment;
	_angle_min = laser_scan_msg.angle_min + yy;
	_laser_ranges = laser_scan_msg.ranges;
	_max_range = laser_scan_msg.range_max;
	for (unsigned int i = 0 ; i < _laser_ranges.size() ; i ++)
	{
		if (_laser_ranges[i] > _max_range)
			_laser_ranges[i] = _max_range;
	}		
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

float RobotPerception::getMapResolution()
{
	return _map_resolution;
} 

/**
 * @brief Returns the occupancy state of a map cell
 * @param i [int] Coordinate x of the map data array 
 * @param j [int] Coordinate y of the map data array 
 * @return int - Map cell occupancy state 
 */
int RobotPerception::getMapCell ( int i, int j ) 
{
	return _map_data[i][j];
}

/**
 * @brief Returns the map data
 * @return int** - Map occupancy data array
 */
int** RobotPerception::getMapData () 
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

float RobotPerception::getRangeMax()
{
	return _max_range;
}

float RobotPerception::getAngleIncrement()
{
	return _increment;
}

float RobotPerception::getAngleMin()
{
	return _angle_min;
}

std::vector<std::string> RobotPerception::getRfidIds()
{
	return _rfid_ids;
}

std::vector<std::string> RobotPerception::getRfidMsgs()
{
	return _rfid_msgs;
}

std::vector<std::vector<float> > RobotPerception::getRfidPose()
{
	return _rfid_pose;
}
