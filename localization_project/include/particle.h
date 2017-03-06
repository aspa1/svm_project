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
		float _previous_x;
		float _previous_y;
		float _previous_theta;
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
		std::vector<float> _distances;
	
	public:
		Particle();
		Particle(unsigned int width, unsigned int height, int** data,
			std::vector<float> ranges, float resolution, int step);
		void randomize(unsigned int width, unsigned int height,
			int** data, float resolution);
		void setPose (float x, float y, float theta);
		void move();
		void getRanges(float angle, unsigned int width, unsigned int height,
			int** data, float resolution, float max_range, int i);
		float rfidSense(std::vector<std::vector<float> > rfid_pose);
		float checkForObstacles(float angle, int** data, float resolution);
		void setParticleWeight(unsigned int width, unsigned int height,
			int** data, float resolution, const std::vector<float>& ranges,
			float max_range, float increment, float angle_min, 
			std::vector<std::vector<float> > rfid_pose, int step, float strictness);
		void calculateMotion(float previous_linear, float previous_angular,
			ros::Duration dt, float a1, float a2);
		float noise(float deviation);
		float getX();
		float getY();
		float getTheta();
		float getWeight();
};

#endif
