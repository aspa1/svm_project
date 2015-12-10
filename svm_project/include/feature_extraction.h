#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <opencv/ml.h>
#include <opencv2/highgui/highgui.hpp>


class FeatureExtraction
{
	private:
		ros::NodeHandle _n;
		//std::vector<cv::Mat> _image;
	public:
		FeatureExtraction();
		std::vector<float> imgPixels(cv::Mat image);	
};

#endif
