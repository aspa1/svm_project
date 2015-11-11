#include "img_folder_handler.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_receiver_client");
  //~ if (argc != 3)
  //~ {
    //~ ROS_INFO("usage: receive_images_from_two_folders Positives Negatives");
    //~ return 1;
  //~ }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<svm_project::images>("image_receiver");
  svm_project::images srv;
  srv.request.positives = ("../samples/positives");
  srv.request.negatives = ("../samples/negatives");
  if (client.call(srv))
  {
	  srv.response.success=true;
	  ROS_INFO("Success: %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service image_receiver");
    return 1;
  }

  return 0;
}
