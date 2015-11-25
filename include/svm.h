#ifndef SVM_H
#define SVM_H

#include "svm_project/trainSvmSrv.h"
#include "svm_project/urlRetrieverSrv.h"
#include <ros/package.h>
#include "boost/filesystem.hpp"  
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <opencv/ml.h>
#include <boost/filesystem.hpp>

	
class SVM {		
	private:	
		ros::NodeHandle n;
		ros::NodeHandle nh;		
		ros::ServiceServer service1;
		ros::ServiceServer service2;
		CvSVM SVM_;
				
	public:
		SVM();
		bool FileExist( const std::string& imagePath );
		bool svmTrain ( svm_project::trainSvmSrv::Request &req, svm_project::trainSvmSrv::Response &res );
		bool svmPredict ( svm_project::urlRetrieverSrv::Request &req, svm_project::urlRetrieverSrv::Response &res );
		void directory(std::string p, float labels[20], float trainingData[20][2], int counter);
		std::string type2str(int type);
		void getAllFilesFromDir (std::string dir, int& counter, float trainingData[20][2]);
		void imgRead(std::string path, int &counter, float &red_percentage, float &bg_percentage);

};



#endif
