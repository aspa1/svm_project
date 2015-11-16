#include "main.h"

int main( int argc, char** argv )
{

    ros::init(argc, argv, "main");
    
	//~ svm_project::images srv;
	//~ srv.request.positives = ("/samples/positives");
	//~ srv.request.negatives = ("/samples/negatives");

	SVM svm1;
	
	ros::spin();

	return 0;

}
