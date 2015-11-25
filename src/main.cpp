#ifndef MAIN
#define MAIN


#include "svm.h"
#include <iostream>



int main( int argc, char** argv )
{

    ros::init(argc, argv, "main");

	SVM svm_;
	
	ros::spin();

	return 0;

}

#endif
