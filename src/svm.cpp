#include "svm.h"

SVM::SVM() {

	service1 = nh.advertiseService("url_retriever", &SVM::url_callback, this);

	ROS_INFO("Service ready to retrieve url");
	
	
	service2 = n.advertiseService("image_receiver", &SVM::image_callback, this);

	ROS_INFO("Service ready to receive images");


}

bool SVM::url_callback ( 	svm_project::urlRetrieverSrv::Request  &req,
							svm_project::urlRetrieverSrv::Response &res 	)
{
	bool flag=false;
	int counter = 0;
	float red_percent, bg_percent, newImage[2];
	
	std::string svmPath = ros::package::getPath("svm_project");
	
	if (boost::filesystem::is_directory(svmPath)) {
		
		
		std::string path1 = svmPath + req.url;
		
		for (boost::filesystem::directory_iterator itr(svmPath); itr!=boost::filesystem::directory_iterator(); ++itr) {
			std::string imgName =(itr->path().filename()).string();
			std::string path2= svmPath + "/" + imgName;
			if (path1==path2) {
				flag=true;
				break;
			}
		}
		if (flag==true) {
			imgRead(path1, counter, red_percent, bg_percent);
		
			newImage[0] = red_percent;
			newImage[1] = bg_percent;
			
			cv::Mat newImageMat(1, 2, CV_32FC1, newImage);	
			float response = SVM_.predict(newImageMat);
			if (response == 1.0) {
				ROS_INFO_STREAM("Image is red");
			}
			else
			{
				ROS_INFO_STREAM("Image is not red");
			}
			res.success = true;	
		}
		else {
			ROS_ERROR("Wrong image name or path");
		}

	}
	else
	{
		ROS_ERROR("Wrong directory");
		res.success = false;
		return 1;
	}

	return true;

}	


bool SVM::image_callback ( 	svm_project::trainSvmSrv::Request  &req,
							svm_project::trainSvmSrv::Response &res 	)
{
	int counter=0;
	float trainingData[20][2], labels[20];
	std::string svmPath = ros::package::getPath("svm_project");
		
	std::string p = svmPath + req.positives;
	std::string n = svmPath + req.negatives;
	
	getAllFilesFromDir(p, counter, trainingData);
	getAllFilesFromDir(n, counter, trainingData);
	
 
	for (int i=0; i<20; i++) {
		if (trainingData[i][0] > trainingData[i][1]) {
			labels[i] = 1.0;
		}
		else {
			labels[i] = -1.0;
		}
	}
	cv::Mat labelsMat(20, 1, CV_32FC1, labels);	
	
	cv::Mat trainingDataMat(20, 2, CV_32FC1, trainingData);	
	
	//~ CvSVMParams params;
	//~ params.svm_type    = CvSVM::C_SVC;
	//~ params.kernel_type = CvSVM::LINEAR;
	//~ params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	
    //~ SVM_.train(trainingDataMat, labelsMat, cv::Mat(), cv::Mat(), params);
    SVM_.train(trainingDataMat, labelsMat);
    
	res.success = true;
	
	
	
	return true;
	
}

void SVM::getAllFilesFromDir (std::string dir, int& counter, float trainingData[20][2]) {
	if (boost::filesystem::is_directory(dir))
	{
		float red_percent, bg_percent;
		for (boost::filesystem::directory_iterator itr(dir); itr!=boost::filesystem::directory_iterator(); ++itr)
		{
			//ROS_INFO_STREAM("Name:" << itr->path().filename()); // display filename only
			std::string imgName =(itr->path().filename()).string();
			//ROS_INFO_STREAM("Image name:" << p1);
			std::string path= dir + "/" + imgName;
			//ROS_INFO_STREAM("Full name:" << p_);
			imgRead(path, counter, red_percent, bg_percent);
			
			trainingData[counter-1][0] = red_percent;
			trainingData[counter-1][1] = bg_percent;
		}
	}
	else {
		ROS_INFO_STREAM ("Failed to retrieve file");
	}
}
	

void SVM::imgRead(std::string path, int &counter, float &red_percentage, float &bg_percentage)
{	
	cv::Mat image;	
	int red_pixel_counter=0;
	int bg_pixel_counter=0;
	
	image = cv::imread(path, CV_LOAD_IMAGE_COLOR);

				
	for (int i=0; i<image.rows; i++) {
		for (int j=0; j<image.cols; j++) {
			int b = image.at<cv::Vec3b> (i,j)[0];
			int g = image.at<cv::Vec3b> (i,j)[1];
			int r = image.at<cv::Vec3b> (i,j)[2];
			if (r > b && r > g) {
				red_pixel_counter++;
			}
			else {
				bg_pixel_counter++;

			}
		}
	}
		
	red_percentage = float(red_pixel_counter) / float(image.rows*image.cols);
	bg_percentage = float(bg_pixel_counter) / float(image.rows*image.cols);
				
	counter++;
			
}	

std::string SVM::type2str(int type) {   //useful for debugging
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
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
  r += (chans+'0');

  return r;
}
				
