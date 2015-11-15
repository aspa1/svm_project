#include "image_processing.h"



bool image_callback ( 	svm_project::images::Request  &req,
						svm_project::images::Response &res 	)
{
	std::string path = ros::package::getPath("svm_project");
	
	std::string p = path + req.positives;
	std::string n = path + req.negatives;
	
	if (boost::filesystem::is_directory(p))
	{
		ROS_INFO_STREAM("Positives:");
		for (boost::filesystem::directory_iterator itr(p); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			ROS_INFO_STREAM("Name:" << itr->path().filename()); // display filename only
		}
	}
	
	if (boost::filesystem::is_directory(n))
	{
		ROS_INFO_STREAM("Negatives:");
		for (boost::filesystem::directory_iterator itr(p); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			ROS_INFO_STREAM("Name:" << itr->path().filename()); // display filename only
		}
	}
	return true;


//~ 
		//~ cv::Mat image;
		//~ image= cv::imread(path + "/samples/positives/1p.jpg", CV_LOAD_IMAGE_COLOR);
		//~ ROS_INFO_STREAM (image.rows);
		//~ cv::imshow( "Display window", image );
		//~ cv::waitKey(5000);
//~ 
		//~ //ROS_INFO_STREAM("Name:" << req.positives << std::endl);
}
			
			
