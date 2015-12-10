#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "robot_perception.h"

class ParticleFilter {
	private:
		ros::NodeHandle _n;
		int particles_number;
		Particle* _particles; 
	
	public:
		ParticleFilter();
};

#endif
