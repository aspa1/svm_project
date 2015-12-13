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
		
		unsigned int _width;
		unsigned int _height;
		int** _data;
		std::vector<float> _ranges;
		float _resolution;
		
		std::vector<float> _particle_ranges;

	
	public:
		Particle();
		Particle( unsigned int width, unsigned int height, int** data, std::vector<float> ranges, float resolution );
		void setParticlePos( int new_x, int new_y, float new_theta );
		void particleRanges(float angle);
		void setParticleWeight();
};

#endif
