#include "particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width,
	unsigned int height, int** data, std::vector<float> ranges, float resolution) 
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
	
	_ranges = ranges;
	_height = height;
	_width = width;
	_data = data;
	_resolution = resolution;
}

void Particle::setParticlePos(int new_x, int new_y, float new_theta) 
{
	_x = new_x;
	_y = new_y;
	_theta = new_theta;
}

void Particle::particleRanges(float angle)
{
	int distance = 1 ;
	float new_x = _x + distance * _resolution * cos(angle);
	float new_y = _y + distance * _resolution * sin(angle);
	

	while ( ((int)new_x <= _width && (int)new_y <= _height) && ((int)new_x > 0 && (int)new_y > 0))
	{
		if (_data[(int)new_x][(int)new_y] == 100)
		{
			_particle_ranges.push_back(distance * _resolution);
			break;
		}
		else
		{
			distance ++;
			new_x = _x + distance * _resolution * cos(angle);
			new_y = _y + distance * _resolution * sin(angle);
		}
	}
}

void Particle::setParticleWeight()
{
	std::vector<float> distances;
	float sum = 0;
	
	particleRanges(_theta);
	particleRanges(PI/2 + _theta);
	particleRanges(PI + _theta);
	particleRanges(3 * PI/2 + _theta);
	
	for ( unsigned int i = 0 ; i < _particle_ranges.size() ; i++ )
	{
		distances.push_back (fabs(_ranges[i]-_particle_ranges[i]));
		sum += distances[i];
		//~ ROS_INFO_STREAM("Distances: ");
		//~ ROS_INFO_STREAM("i = " << i << " " << distances[i]);
	}
	
	_weight = 1/(sum + 1);
	//~ ROS_INFO_STREAM("weight: " << " " << _weight);
}
