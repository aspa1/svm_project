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
		float _x;
		float _y;
		float _theta;
		float _dx;
		float _dy;
		float _dtheta;
		float _weight;		
		float* _particle_ranges;
		float _linear;
		float _angular;
	
	public:
		Particle();
		Particle(unsigned int width, unsigned int height, int** data,
			std::vector<float> ranges, float resolution);
		void move();
		void sense(float angle, unsigned int width, unsigned int height,
			int** data, float resolution, int i);
		void calculateMotion(float previous_linear, float previous_angular,
			ros::Duration dt, float a1, float a2);
		float noise(float deviation);
		void setParticleWeight(unsigned int width, unsigned int height,
			int** data, float resolution, std::vector<float> ranges,
			float max_range, float increment, float angle_min);
		float getX();
		float getY();
		float getWeight();
};

#endif
