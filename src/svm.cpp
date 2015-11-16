#include "svm.h"

SVM::SVM() {


	client = n.serviceClient<svm_project::images>("image_receiver");
	ROS_INFO("1");

	service = n.advertiseService("image_receiver", &SVM::image_callback, this);
	ROS_INFO("2");
	

	ROS_INFO("Service ready to receive images");

	srv.request.positives = ("/samples/positives");
	srv.request.negatives = ("/samples/negatives");

	if (client.call(srv))
	{
		ROS_INFO_STREAM("3");
	  if(client.waitForExistence())
		{
		  srv.response.success=true;
		  ROS_INFO("Success: %d", srv.response.success);
		}
		
	}
	else
	{
		ROS_INFO_STREAM("4");
		ROS_ERROR("Failed to call service image_receiver");
		srv.response.success= false;
		
	}
	ROS_INFO_STREAM("5");
	
}



bool SVM::image_callback ( 	svm_project::images::Request  &req,
						svm_project::images::Response &res 	)
{
	ROS_INFO_STREAM("callback");
	
	//~ srv.request.positives = ("/samples/positives");
	//~ srv.request.negatives = ("/samples/negatives");
	
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
			
			
