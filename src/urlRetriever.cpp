#include "urlRetriever.h"


URL::URL() {
	
	service = n.advertiseService("url_retriever", &URL::url_callback, this);

	ROS_INFO("Service ready to retrieve url");

}

bool URL::url_callback ( 	svm_project::urlRetrieverSrv::Request  &req,
							svm_project::urlRetrieverSrv::Response &res 	)
{

	try
	{
		resource = r.get(req.url); 
		res.success= true;
	}
	catch (resource_retriever::Exception& e)
	  {
		ROS_ERROR("Failed to retrieve file: %s", e.what());
		res.success= false;
		return 1;
	  }

	FILE* f = fopen("out.jpg", "w");
	fwrite(resource.data.get(), resource.size, 1, f);
	fclose(f);
	  
	ROS_INFO("Wrote data from url to out.jpg");
	return true;

}	
