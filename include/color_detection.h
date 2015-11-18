#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <ros/package.h>
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>



	
class Color_detection {		
	private:	
		ros::NodeHandle n;
		void image_callback ( const sensor_msgs::ImageConstPtr& color_img);
		
	public:
		Color_detection();
		

};



#endif
