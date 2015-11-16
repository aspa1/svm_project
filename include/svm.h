#ifndef SVM_H
#define SVM_H

#include "svm_project/images.h"
#include <ros/package.h>
#include "boost/filesystem.hpp"  
#include "ros/ros.h"
//~ #include <opencv2/highgui/highgui.hpp>


	
class SVM {		
	private:	
		ros::NodeHandle n;
		bool image_callback ( svm_project::images::Request &req, svm_project::images::Response &res );
		ros::ServiceServer service;
		ros::ServiceClient client;
		svm_project::images srv;
		
	public:
		SVM();
		

};



#endif
