#ifndef IMG_READ_H
#define IMG_READ_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <opencv/ml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


class ImgRead
{
	private:
		ros::NodeHandle _n;

	public:
		ImgRead();
		//~ bool fileExist( const std::string& imagePath );
		cv::Mat imgRead(std::string path);
		std::string type2str(int type);		
		
};

#endif
