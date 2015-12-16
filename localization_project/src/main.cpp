#include "particle_filter.h"

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "main");
	ParticleFilter particle_filter;
	ros::Rate loop_rate(10);
	ros::spin(); 
	loop_rate.sleep();
	return 0;
}
