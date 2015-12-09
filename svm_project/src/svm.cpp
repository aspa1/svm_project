#include "svm.h"

SVM::SVM() 
{
	_uri_retriever_service = _n.advertiseService("uri_retriever", &SVM::svmPredict, this);
	ROS_INFO("Service ready to test an image from uri");
	
	_image_receiver_service = _n.advertiseService("image_receiver", &SVM::svmTrain, this);
	ROS_INFO("Service ready to receive images and train svm");
}

bool SVM::fileExist( const std::string& name )
{
    return boost::filesystem::exists(name);
}

bool SVM::svmPredict ( 	
	svm_project::urlRetrieverSrv::Request  &req,
	svm_project::urlRetrieverSrv::Response &res 	
	)
{
	if (fileExist(req.url) == true) 
	{
		std::vector<float> features = imgRead(req.url);
		float *feature_vector = new float[features.size()];
		for (unsigned int i = 0 ; i < features.size() ; i++)
		{
			feature_vector[i] = features[i];
		}	
		
		cv::Mat new_image_mat(1, 2, CV_32FC1, feature_vector);	
		float response = _svm.predict(new_image_mat);
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


bool SVM::svmTrain ( 	
	svm_project::trainSvmSrv::Request  &req,
	svm_project::trainSvmSrv::Response &res 	
	)
{
	std::string p = req.positives;
	std::string n = req.negatives;
	
	std::vector<std::vector<float> > positive_features = getAllFilesFromDir(p);
	std::vector<std::vector<float> > negative_features = getAllFilesFromDir(n);
	
	if(positive_features.size() == 0)
	{
		return true;
	}
	
	// Allocation of training data
	float **training_data;
	unsigned int data_size = positive_features.size() + negative_features.size();
	training_data = new float*[data_size];
	for (unsigned int i = 0 ; i < data_size ; i++)
	{
		training_data[i] = new float[positive_features[0].size()];
	}
	
	float * labels = new float[data_size];
	
	for (unsigned int i = 0 ; i < positive_features.size() ; i++) 
	{	
		for (unsigned int j = 0 ; j < positive_features[i].size() ; j++)
		{
			training_data[i][j] = positive_features[i][j];
		}
		labels[i] = 1.0;
	}
	for (unsigned int i = 0 ; i < negative_features.size() ; i++) 
	{	
		for (unsigned int j = 0 ; j < negative_features[i].size() ; j++)
		{
			training_data[i][j] = negative_features[i][j];
		}
		labels[i] = -1.0;
	}

	cv::Mat labels_mat(20, 1, CV_32FC1, labels);	
	cv::Mat training_data_mat(20, 2, CV_32FC1, training_data);	
	
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

std::vector<std::vector<float> > SVM::getAllFilesFromDir (std::string dir) 
{
	std::vector<std::vector<float> > training_data;
	if (fileExist(dir) == true)
	{
		float red_percent = 0;
		float bg_percent = 0;
		for (boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); ++itr)
		{
			std::string img_name = (itr->path().filename()).string();
			std::string path= dir + "/" + img_name;
			training_data.push_back(imgRead(path));
		}
	}
	else 
	{
		ROS_INFO_STREAM ("Wrong path..Failed to retrieve file");
	}
	return training_data;
}

std::vector<float> SVM::imgRead(std::string path)
{	
	cv::Mat image;	
	
	image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
	
	int red_pixel_counter = 0;
	int bg_pixel_counter = 0;
			
	for (int i = 0 ; i < image.rows ; i++) 
	{
		for (int j = 0 ; j < image.cols ; j++) 
		{
			int b = image.at<cv::Vec3b> (i,j)[0];
			int g = image.at<cv::Vec3b> (i,j)[1];
			int r = image.at<cv::Vec3b> (i,j)[2];
			if (r > b && r > g) 
			{
				red_pixel_counter++;
			}
			else 
			{
				bg_pixel_counter++;
			}
		}
	}
		
	std::vector<float> ret;
	ret.push_back( float(red_pixel_counter) / float(image.rows * image.cols) );
	ret.push_back( float(bg_pixel_counter) / float(image.rows * image.cols) );
	return ret;
}	

std::string SVM::type2str(int type) 
{   //useful for debugging
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) 
  {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}
				
