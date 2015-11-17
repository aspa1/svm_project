#ifndef URL_H
#define URL_H

#include "svm_project/urlRetrieverSrv.h"
#include <ros/package.h>
#include "ros/ros.h"
#include "resource_retriever/retriever.h"
#include <ros/console.h>

	
class URL {		
		
		ros::NodeHandle n;
		bool url_callback ( svm_project::urlRetrieverSrv::Request &req, svm_project::urlRetrieverSrv::Response &res );
		ros::ServiceServer service;
		resource_retriever::Retriever r;
		resource_retriever::MemoryResource resource;
				
	public:
		URL();
		

};



#endif
