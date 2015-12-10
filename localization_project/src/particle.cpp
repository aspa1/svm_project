#include "particle.h"

Particle::Particle() {}

Particle::Particle( unsigned int width, unsigned int height, int** data) 
{	
	_x = std::rand() % ( width + 1 );
	_y = std::rand() % ( height + 1 );
	 
	//~ while (data[_x][_y] == - 1)
	//~ {
		//~ _x = std::rand() % ( width + 1 );
		//~ _y = std::rand() % ( height + 1 );
	//~ }
	
	_theta = static_cast <float> (rand()) / static_cast <float> 
		(RAND_MAX/2*PI);
	ROS_INFO_STREAM ("width = "<< " " << width << " " << "height = " << " " <<height);
	ROS_INFO_STREAM ("x = "<< " " << _x << " " << "y = "<< _y << " " << " theta = " << " " << _theta);
}

void Particle::setParticlePos(int new_x, int new_y, float new_theta) 
{
	_x = new_x;
	_y = new_y;
	_theta = new_theta;
}

void Particle::setParticleWeight()
{
	
}
