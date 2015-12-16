#include "particle_filter.h"

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "main");
	ParticleFilter particle_filter;
	ros::spin(); 
	return 0;
}
