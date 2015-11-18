#ifndef SVM_H
#define SVM_H

#include "svm_project/trainSvmSrv.h"
#include "svm_project/urlRetrieverSrv.h"
#include <ros/package.h>
#include "boost/filesystem.hpp"  
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include "resource_retriever/retriever.h"
#include <ros/console.h>
#include <sensor_msgs/Image.h>

	
class SVM {		
	private:	
		ros::NodeHandle n;
		ros::NodeHandle nh;		
		bool image_callback ( svm_project::trainSvmSrv::Request &req, svm_project::trainSvmSrv::Response &res );
		bool url_callback ( svm_project::urlRetrieverSrv::Request &req, svm_project::urlRetrieverSrv::Response &res );
		ros::ServiceServer service1;
		ros::ServiceServer service2;
		//~ resource_retriever::Retriever r;
		//~ resource_retriever::MemoryResource resource;
				
	public:
		SVM();
		

};



#endif
