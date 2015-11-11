#include "image_processing.h"



bool image_callback ( 	svm_project::images::Request  &req,
						svm_project::images::Response &res 	)
{
	ROS_INFO("Positives %s", req.positives.c_str());
	ROS_INFO("Negatives %s", req.negatives.c_str());
	
	cv::Mat image;
    image= cv::imread("/home/aspa/catkin_ws/src/svm_project/samples/positives/1p.jpg", CV_LOAD_IMAGE_COLOR);
    ROS_INFO_STREAM (image.rows);
    cv::imshow( "Display window", image );
    cv::waitKey(5000);
    
	//ROS_INFO_STREAM("Name:" << req.positives << std::endl);
}
			
			
