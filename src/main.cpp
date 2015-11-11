#include "main.h"


int main( int argc, char** argv )
{

    ros::init(argc, argv, "main");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("image_receiver", image_callback);
	ROS_INFO("Service ready to receive images");
	ros::spin();

	return 0;

}
    
    //req.positive_image image;
    //image = imread(../samples/positives, CV_LOAD_IMAGE_COLOR);
