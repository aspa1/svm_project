#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "robot_perception.h"

#include "localization_project/particleInitSrv.h"


class ParticleFilter {
	private:
		ros::NodeHandle _n;
		RobotPerception robot_percept;
		ros::ServiceServer _particle_initialization_service;
		int particles_number;
		std::vector<Particle> _particles; 
	
	public:
		ParticleFilter();
		bool particlesInitCallback ( 
			localization_project::particleInitSrv::Request& req,
			localization_project::particleInitSrv::Response& res
		);
};

#endif
