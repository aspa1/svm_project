#ifndef SVM_H
#define SVM_H

#include "svm_project/trainSvmSrv.h"
#include <ros/package.h>
#include "boost/filesystem.hpp"  
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>


	
class SVM {		
	private:	
		ros::NodeHandle n;
		bool image_callback ( svm_project::trainSvmSrv::Request &req, svm_project::trainSvmSrv::Response &res );
		ros::ServiceServer service;
		ros::ServiceClient client;
		//svm_project::trainSvmSrv srv;
		
	public:
		SVM();
		

};



#endif
