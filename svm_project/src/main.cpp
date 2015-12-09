#include "svm.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "main");
	SVM svm;
	ros::spin();
	return 0;
}
