#include "feature_extraction.h"

FeatureExtraction::FeatureExtraction()
{
	
}

/** 
 * @brief Finds and return the matrix of image's pixels
 */
std::vector<float> FeatureExtraction::imgPixels(cv::Mat img)
{		
	//~ std::vector<cv::Mat> image = img;
	cv::Mat image = img;
	int red_pixel_counter = 0;
	int bg_pixel_counter = 0;
	
			
	for (unsigned int i = 0 ; i < image.rows ; i++) 
	{
		for (unsigned int j = 0 ; j < image.cols ; j++) 
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
		
	std::vector<float> pixels_percentage;
	pixels_percentage.push_back( float(red_pixel_counter) / 
		float(image.rows * image.cols) );
	pixels_percentage.push_back( float(bg_pixel_counter) / 
		float(image.rows * image.cols) );
	return pixels_percentage;
}	
