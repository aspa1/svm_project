#include "image_processing.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

using namespace cv;

bool image_callback ( 	svm_project::images::Request  &req,
						svm_project::images::Response &res 	)
{
	ROS_INFO("Positives %s", req.positives.c_str());
	ROS_INFO("Negatives %s", req.negatives.c_str());
	
	cv::Mat image;
    //~ image = imread(req.positives, CV_LOAD_IMAGE_COLOR);
    //image = (cv::imread("/home/aspa/catkin_ws/src/svm_project/samples/positives/1.jpg", 1)).c_str();
    image= cv::imread("/home/aspa/catkin_ws/src/svm_project/samples/positives/1.jpg", 0);
    //imshow( "Display window", image );
    //image = cv::imshow("/home/aspa/catkin_ws/src/svm_project/samples/positives/1.jpg", 1);
    cv::waitKey(5000);
    
	//ROS_INFO_STREAM("Name:" << req.positives << std::endl);
}
			
			
