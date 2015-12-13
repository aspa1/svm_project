#ifndef PARTICLE_H
#define PARTICLE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <cstdlib>
#include <math.h>

#define PI 3.14159265359

class Particle
{
	private:
		int _x;
		int _y;
		float _theta;
		float _weight;
	
	public:
		Particle();
		Particle( unsigned int width, unsigned int height, int** data, std::vector<float> ranges );
		void setParticlePos( int new_x, int new_y, float new_theta );
		void setParticleWeight();
};

#endif
