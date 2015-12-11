#include "particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	_particle_initialization_service = _n.advertiseService (
		"particle_initialization",
		&ParticleFilter::particlesInitCallback, this);
	ROS_INFO("Service ready to initialize particles");
}

bool ParticleFilter::particlesInitCallback (
	localization_project::particleInitSrv::Request& req,
	localization_project::particleInitSrv::Response& res
	)
{
	
	if(!_n.getParam("/particles_number", particles_number))
	{
		ROS_ERROR("Particles number param does not exist");
	}	
	
	ROS_INFO_STREAM ("ParticleFilter_width ="<< " " << robot_percept.getMapWidth() << " " << "ParticleFilter_height ="<< " " << robot_percept.getMapHeight());
	
	//~ for (unsigned int i = 0 ; i < robot_percept.getMapWidth() ; i++ )
	//~ {
		//~ for (unsigned int j = 0 ; j < robot_percept.getMapHeight() ; j++ ) 
		//~ {
			//~ ROS_INFO_STREAM (" i = " << " " << i << " " << "j = " << " " << j << " " <<" Map data = " << " " << robot_percept.getMapCell(i,j));
		//~ }
	//~ }
	
	
	for (unsigned int i = 0 ; i < particles_number ; i++ )
	{
		Particle particle( robot_percept.getMapWidth(),
			robot_percept.getMapHeight(), robot_percept.getMapData(),
			robot_percept.getLaserRanges());
		_particles.push_back(particle);
	}
	return true;
}
