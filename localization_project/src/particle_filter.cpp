#include "particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	RobotPerception robot_percept;
	
	if(!_n.getParam("/particles_number", particles_number))
	{
		ROS_ERROR("Particles number param does not exist");
	}	
	_particles = new Particle[particles_number];
	
	ROS_INFO_STREAM ("ParticleFilter_width ="<< " " << robot_percept.getMapWidth() << " " << "ParticleFilter_height ="<< " " << robot_percept.getMapHeight());
	
	for (unsigned int i = 0 ; i < particles_number ; i++ )
	{
		Particle particle( robot_percept.getMapWidth(),
			robot_percept.getMapHeight(), robot_percept.getMapData());
		_particles[i] = particle;
	}
}
