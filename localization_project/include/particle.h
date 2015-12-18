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
		std::vector<float> _particle_ranges;
	
	public:
		Particle();
		Particle( unsigned int width, unsigned int height, int** data );
		//~ void setPose( int new_x, int new_y, float new_theta, unsigned int width, unsigned int height );
		void move(ros::Duration dt, float linear, float angular, float resolution);
		void sense(float angle, unsigned int width, unsigned int height, int** data, float resolution);
		void setParticleWeight(unsigned int width, unsigned int height, int** data, float resolution, std::vector<float> ranges);
		int getX();
		int getY();
		float getWeight();
};

#endif
