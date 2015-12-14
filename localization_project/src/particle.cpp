#include "particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width, unsigned int height, int** data) 
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
	//~ ROS_INFO_STREAM ("Initial x = "<< " " << _x << " " << "y = "<< _y << " " << " theta = " << " " << _theta);
	
	//~ _x = 500;
	//~ _y = 500;
	//~ _theta = 0;
}

void Particle::setParticlePos(int new_x, int new_y, float new_theta) 
{
	_x = new_x;
	_y = new_y;
	_theta = new_theta;
}

void Particle::particleRanges(float angle, unsigned int width, unsigned int height, int** data, float resolution)
{
	int distance = 1 ;
	float new_x = _x + distance * cos(angle);
	float new_y = _y + distance * sin(angle);
	
	//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
	while ( ((int)new_x <= width && (int)new_y <= height) && ((int)new_x > 0 && (int)new_y > 0))
	{
		//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
		if (data[(int)(new_x)][(int)(new_y)] > 50)
		{
			_particle_ranges.push_back(distance * resolution);
			return;
		}
		else
		{
			distance ++;
			new_x = _x + distance * cos(angle);
			new_y = _y + distance * sin(angle);
		}
	}
	_particle_ranges.push_back( (distance-1) * resolution);
}

void Particle::setParticleWeight(unsigned int width, unsigned int height, int** data, float resolution, std::vector<float> ranges)
{
	std::vector<float> distances;
	float sum = 0;
	
	particleRanges(PI + _theta, width, height, data, resolution);
	particleRanges(3 * PI/2 + _theta, width, height, data, resolution);
	particleRanges(_theta, width, height, data, resolution);
	particleRanges(PI/2 + _theta, width, height, data, resolution);
		
	//~ ROS_INFO_STREAM("Particle_ranges_size" << " " <<_particle_ranges.size());
	for ( unsigned int i = 0 ; i < _particle_ranges.size() ; i++ )
	{
		distances.push_back (fabs(ranges[i]-_particle_ranges[i]));
		sum += 0.8 * distances[i];
		//~ ROS_INFO_STREAM("Distances: ");
		//~ ROS_INFO_STREAM("i = " << i << " " << distances[i]);
	}
	
	_weight = 1/(sum + 1);
	ROS_INFO_STREAM("Weight: " << " " << _weight);
}
