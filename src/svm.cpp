#include "svm.h"

SVM::SVM() {

	service = n.advertiseService("image_receiver", &SVM::image_callback, this);

	ROS_INFO("Service ready to receive images");


}



bool SVM::image_callback ( 	svm_project::trainSvmSrv::Request  &req,
						svm_project::trainSvmSrv::Response &res 	)
{
	
	std::string path = ros::package::getPath("svm_project");
	
	//ROS_INFO_STREAM("Path:" << path);
	
	std::string p = path + req.positives;
	std::string n = path + req.negatives;
	
	if (boost::filesystem::is_directory(p))
	{
		cv::Mat image;
		ROS_INFO_STREAM("Positives:");
		for (boost::filesystem::directory_iterator itr(p); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			ROS_INFO_STREAM("Name:" << itr->path().filename()); // display filename only
			std::string p1 =(itr->path().filename()).string();
			//ROS_INFO_STREAM("Image name:" << p1);
			std::string p_= p + "/" + p1;
			ROS_INFO_STREAM("Full name:" << p_);
			image = cv::imread(p_ , CV_LOAD_IMAGE_COLOR);
			ROS_INFO_STREAM (image.rows);
			//cv::imshow( "Display window", image );
			cv::waitKey(5);
		}
	}
	
	if (boost::filesystem::is_directory(n))
	{
		cv::Mat image;
		ROS_INFO_STREAM("Negatives:");
		for (boost::filesystem::directory_iterator itr(n); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			ROS_INFO_STREAM("Name:" << itr->path().filename()); // display filename only
			std::string n1 =(itr->path().filename()).string();
			//ROS_INFO_STREAM("Image name:" << n1);
			std::string n_= n + "/" + n1;
			ROS_INFO_STREAM("Full name:" << n_);
			image = cv::imread(n_ , CV_LOAD_IMAGE_COLOR);
			ROS_INFO_STREAM (image.rows);
			//cv::imshow( "Display window", image );
			cv::waitKey(5);
		}
	}
	res.success = true;
	
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
			
			
