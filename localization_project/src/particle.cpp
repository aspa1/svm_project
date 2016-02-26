#include "particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width, unsigned int height, int** data) 
{	
	_x = std::rand() % ( width );
	_y = std::rand() % ( height );
	 
	while ( (data[_x][_y] == - 1) || (data[_x][_y] > 50) )
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

//~ void Particle::setPose(int new_x, int new_y, float new_theta, unsigned int width, unsigned int height) 
//~ {
	//~ if (new_x >= width || new_y >= height || new_theta >= 2*PI )
	//~ {
		//~ ROS_ERROR ("Coordinates out of bound");
		//~ return;
	//~ }
	//~ if (new_x < 0 || new_y < 0 || new_theta < 0)
	//~ {
		//~ ROS_ERROR ("Coordinates cannot be negative");
		//~ return;
	//~ }
	//~ _x = new_x;
	//~ _y = new_y;
	//~ _theta = new_theta;
//~ }

void Particle::move(int dx, int dy, float dtheta, float resolution)
{
    //~ ROS_INFO_STREAM("x1 = " << _x << " y1 = " << _y << " theta1 = " << _theta);
    //~ ROS_INFO_STREAM("dx = " << dx << " dy = " << dy << " dtheta = " << dtheta);
    //~ ROS_INFO_STREAM("int dx = " << (int)dx << " int dy = " << (int)dy << " dtheta = " << dtheta);
    _x += dx;
    _y += dy;
    _theta += dtheta;
    //ROS_INFO_STREAM("new x "<< _x << "  new y " << _y << " new theta " << _theta);
    //~ ROS_INFO_STREAM ("dt" << " " << dt.toSec() << " " << "linear" << " " << linear << " " << "angular" << " " << angular);
}

void Particle::sense(float angle, unsigned int width, unsigned int height, int** data, float resolution)
{
	int distance = 1 ;
	float new_x = _x + distance * cos(angle);
	float new_y = _y + distance * sin(angle);
	
	//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
	while ( ((int)new_x < width && (int)new_y < height) && ((int)new_x >= 0 && (int)new_y >= 0))
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
	
	sense(PI + _theta, width, height, data, resolution);
	sense(3 * PI/2 + _theta, width, height, data, resolution);
	sense(_theta, width, height, data, resolution);
	sense(PI/2 + _theta, width, height, data, resolution);
		
	//~ ROS_INFO_STREAM("Particle_ranges_size" << " " <<_particle_ranges.size());
	for ( unsigned int i = 0 ; i < _particle_ranges.size() ; i++ )
	{
		distances.push_back (fabs(ranges[i]-_particle_ranges[i]));
		sum += 0.8 * distances[i];
		//~ ROS_INFO_STREAM("Distances: ");
		//~ ROS_INFO_STREAM("i = " << i << " " << distances[i]);
	}
	
	_weight = 1/(sum + 1);
	//~ ROS_INFO_STREAM("Weight: " << " " << _weight);
}

int Particle::getX()
{
	return _x;
}

int Particle::getY() 
{
	return _y;
}

float Particle::getWeight()
{
	return _weight;
}
