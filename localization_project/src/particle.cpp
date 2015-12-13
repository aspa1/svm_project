#include "particle.h"

Particle::Particle() {}

Particle::Particle(unsigned int width,
	unsigned int height, int** data, std::vector<float> ranges) 
{	
	_x = std::rand() % ( width );
	_y = std::rand() % ( height );
	 
	while ( (data[_x][_y] == - 1) || (data[_x][_y] == 100) )
	{
		_x = std::rand() % ( width );
		_y = std::rand() % ( height );
	}


	_theta = static_cast <float> (rand()) / static_cast <float> 
		(RAND_MAX/2*PI);
	//~ ROS_INFO_STREAM ("x = "<< " " << _x << " " << "y = "<< _y << " " << " theta = " << " " << _theta);
}

void Particle::setParticlePos(int new_x, int new_y, float new_theta) 
{
	_x = new_x;
	_y = new_y;
	_theta = new_theta;
}

void Particle::setParticleWeight()
{
	//degrees = rad * PI / 180.0;	
}
