#include "img_read.h"

ImgRead::ImgRead()
{
	
}

/** 
 * @brief Reads and return a vector of images from a dir 
 */
std::vector<cv::Mat> ImgRead::getImgFromDir(std::string dir)
{
	std::vector<cv::Mat> image;
	for (boost::filesystem::directory_iterator itr(dir); itr != 
			boost::filesystem::directory_iterator(); ++itr)
		{
			std::string img_name = (itr->path().filename()).string();
			std::string path= dir + "/" + img_name;
			image.push_back(cv::imread(path, CV_LOAD_IMAGE_COLOR));
		}	
	return image;	
}

cv::Mat ImgRead::getImg(std::string path)
{	
	cv::Mat image;	
	image = cv::imread(path, CV_LOAD_IMAGE_COLOR);	
		
	return image;	
}

/** 
 * @brief Reads and return an image
 */
//~ std::vector<cv::Mat> ImgRead::imgRead(std::string path)
//~ {	
	//~ std::vector<cv::Mat> image;	
	//~ for (unsigned int i = 0 ; i < image.size() ; i++)
	//~ {
		//~ image[i].push_back(cv::imread(path, CV_LOAD_IMAGE_COLOR));
		//~ ROS_INFO_STREAM("hello");
	//~ }	
		//~ 
	//~ return image;	
//~ }

/** 
 * @brief Finds the type of an image
 */
std::string ImgRead::type2str(int type) 
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
