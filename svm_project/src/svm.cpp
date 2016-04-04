#include "svm.h"

/** 
 * @brief Default constructor. Creates the services.
 */
SVM::SVM() 
{	
	_uri_retriever_service = _n.advertiseService("uri_retriever", 
		&SVM::svmPredict, this);
	ROS_INFO("Service ready to test an image from uri");
	
	_image_receiver_service = _n.advertiseService("image_receiver", 
		&SVM::svmTrain, this);
	ROS_INFO("Service ready to receive images and train svm");
}

/** 
 * @brief Finds if a file exists or not
 */
bool SVM::fileExist( const std::string& name )
{
    return boost::filesystem::exists(name);
}

/**
 * @brief Serves the uri service callback
 * @param req [svm_project::urlRetrieverSrv::Request&] The uri Retriever
 * 	 service request
 * @param res [svm_project::urlRetrieverSrv::Response&] The uri 
 * 	Retriever service response
 * @return bool - The success status of the call
 */
bool SVM::svmPredict ( 	
	svm_project::urlRetrieverSrv::Request  &req,
	svm_project::urlRetrieverSrv::Response &res 	
	)
{
	std::string url = req.url;
	
	if (fileExist(url) == true) 
	{	
		cv::Mat input = cv::imread(url, 0); //Load as grayscale
		cv::SiftFeatureDetector detector;
		std::vector<cv::KeyPoint> keypoints;
		detector.detect(input, keypoints);

		// Add results to image and save.
		cv::Mat output;
		cv::drawKeypoints(input, keypoints, output);
		cv::imwrite("/home/chrisa/catkin_ws/src/thesis/sift_result.bmp", output);
		//~ imshow( "Display window", output );                   // Show our image inside it.
//~ 
		//~ cv::waitKey(0);  
    
		cv::Mat image = img.getImg(url);
		std::vector<float> features = f.imgPixels(image);

		float *feature_vector = new float[features.size()];
		for (unsigned int i = 0 ; i < features.size() ; i++)
		{
			feature_vector[i] = features[i];
		}	
		
		ROS_INFO_STREAM("features:");
		for (unsigned int i = 0 ; i < features.size() ; i++)
		{
			ROS_INFO_STREAM(feature_vector[i]);
		}	
		
		cv::Mat new_image_mat(1, features.size(), CV_32FC1, 
			feature_vector);	
		
		float response = _svm.predict(new_image_mat);
		
		ROS_INFO_STREAM("response:"<< response);
		
		if (response == 1.0) 
		{
			ROS_INFO_STREAM("Image is red");
		}
		else
		{
			ROS_INFO_STREAM("Image is not red");
		}
		delete [] feature_vector;
		res.success = true;	
	}
	else 
	{
		ROS_ERROR("Wrong image name or path");
		res.success = false;
		return 1;
	}
    
	return true;
}	

/**
 * @brief Serves the image service callback
 * @param req [svm_project::trainSvmSrv::Request&] The image receiver 
 * 	service request
 * @param res [svm_project::trainSvmSrv::Response&] The image receiver 
 * service response
 * @return bool - The success status of the call
 */
bool SVM::svmTrain ( 
	svm_project::trainSvmSrv::Request &req, 
	svm_project::trainSvmSrv::Response &res )
{
	std::string p = req.positives;
	std::string n = req.negatives;
	
	std::vector<std::vector<float> > positive_features;
	positive_features = getData(p);
	std::vector<std::vector<float> > negative_features;
	negative_features = getData(n);
	
	for (unsigned int i = 0 ; i < positive_features.size() ; i++) 
	{	
		for (unsigned int j = 0 ; j < positive_features[i].size() ; j++)
		{
			ROS_INFO_STREAM(positive_features[i][j]);
		}
	}
	for (unsigned int i = 0 ; i < positive_features.size() ; i++) 
	{	
		for (unsigned int j = 0 ; j < positive_features[i].size() ; j++)
		{
			ROS_INFO_STREAM(positive_features[i][j]);
		}
	}
	
	if(positive_features.size() == 0)
	{
		return true;
	}
	
	// Allocation of training data
	float **training_data;
	
	unsigned int data_size = positive_features.size() + 
		negative_features.size();
	training_data = new float*[data_size];
	ROS_INFO_STREAM(data_size);
	
	for (unsigned int i = 0 ; i < data_size ; i++)
	{
		training_data[i] = new float[positive_features[0].size()];
	}
	
	float *labels = new float[data_size];
	
	ROS_INFO_STREAM("Training data positive:");
	for (unsigned int i = 0 ; i < positive_features.size() ; i++) 
	{	
		for (unsigned int j = 0 ; j < positive_features[i].size() ; j++)
		{
			training_data[i][j] = positive_features[i][j];
		}
	}      
      
	ROS_INFO_STREAM("Training data negative:");
	for (unsigned int i = positive_features.size() ; i< data_size ; i++)
	{	
		for (unsigned int j = 0 ; j < negative_features[i-
			positive_features.size()].size() ; j++)
		{
			training_data[i][j] = negative_features[i-
				positive_features.size()][j];
		}
	}
	
	for (unsigned int i = 0 ; i < positive_features.size() ; i++) 
	{	
		 labels[i] = 1.0;
	}
	for (unsigned int i = positive_features.size() ; i < data_size ; i++) 
	{	
		 labels[i] = -1.0;
	}
	
	for (unsigned int i = 0 ; i< data_size ; i++)
	{	
		for (unsigned int j = 0 ; j < 2 ; j++)
		{
			
			ROS_INFO_STREAM(training_data[i][j]);
			
		}
		ROS_INFO_STREAM(labels[i]);
	}
	cv::Mat labels_mat(data_size,1, CV_32FC1, labels);	
	cv::Mat training_data_mat(data_size, positive_features[0].size(), 
		CV_32FC1);
		
	for (unsigned int i = 0 ; i < data_size ; i++)
	{
		for (unsigned int j = 0 ; j < positive_features[0].size() ; j++)
		{
			training_data_mat.at<float>(i,j) = training_data[i][j];	
		}
	}
	
    _svm.train(training_data_mat, labels_mat);
    
    
    for (unsigned int i = 0 ; i < data_size ; i++)
    {
		delete [] training_data[i];
	}
	delete [] training_data;
	delete [] labels;
    
	res.success = true;
	return true;
}


/** 
 * @brief Finds and return the matrix of image's pixels
 */
std::vector<std::vector<float> > SVM::getData (std::string dir) 
{
	std::vector<std::vector<float> > training_data;
	if (fileExist(dir) == true)
	{
		std::vector<cv::Mat> image = img.getImgFromDir(dir);
		for (unsigned int i = 0 ; i < image.size() ; i++)
		{
			training_data.push_back(f.imgPixels(image[i]));
		}	
	}
	else 
	{
		ROS_INFO_STREAM ("Wrong path..Failed to retrieve file");
	}
	return training_data;
}



				
