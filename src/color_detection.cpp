#include "color_detection.h"



Color_detection::Color_detection() {

}

void Color_detection::image_callback(const sensor_msgs::ImageConstPtr& color_img) {

	cv_bridge::CvImagePtr img_ptr;
	cv::Mat img_rgb;
	try {
		img_ptr = cv_bridge::toCvCopy(color_img, sensor_msgs::image_encodings::BGR8);
		img_rgb = img_ptr->image;
	}
	catch (cv_bridge::Exceptions& e) {
		ROS_ERROR("cv_bridge exception: \%s", e.what());
		return;
	}
	
	cv::Mat img_hsv;
	cv::cvtColor(img_rgb, img_hsv, CV_BGR2HSV);
	cv::imshow("HSV image", img_hsv)
	cv::waitKey(0);
	
	//~ cv::Mat img_binary;
	//~ cv::Scalar min_vals(165,240,100);
	//~ cv::Scalar max_vals(179,255,175);
	//~ cv::inRange(img_hsv, min_vals, max_vals, img_binary);
	//~ 
	//~ unsigned int n_pixels_thr = 0;


}	
