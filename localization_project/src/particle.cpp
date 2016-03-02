#include "particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width, unsigned int height, int** data, std::vector<float> ranges, float resolution) 
{	
	_x = std::rand() % (width) * resolution;
	_y = std::rand() % (height) * resolution;
	//~ ROS_INFO_STREAM("x = " << _x << " y = " << _y);
	//~ ROS_INFO_STREAM("(int)(_x / resolution) " << (int)(_x / resolution) << " (int)(_y / resolution) = " << (int)(_y / resolution));
	 
	while ( (data[(int)(_x / resolution)][(int)(_y / resolution)] == - 1) || (data[(int)(_x / resolution)][(int)(_y / resolution)] > 50) )
	{
		_x = std::rand() % (width) * resolution;
		_y = std::rand() % (height) * resolution;
	}

	//~ _theta = static_cast <float> (rand()) / static_cast <float> 
		//~ (RAND_MAX/2*PI);
	_theta = 0;
	//~ ROS_INFO_STREAM("theta = " << _theta);

		
	_particle_ranges = new float[ranges.size()];	
	//~ ROS_INFO_STREAM ("Initial x = "<< " " << _x << " " << "y = "<< _y << " " << " theta = " << " " << _theta);
	
	//~ _x = 15;
	//~ _y = 15;
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

void Particle::move(float dx, float dy, float dtheta, float resolution)
{
    //~ ROS_INFO_STREAM("x1 = " << _x << " y1 = " << _y << " theta1 = " << _theta);
    //~ ROS_INFO_STREAM("dx = " << dx << " dy = " << dy << " dtheta = " << dtheta);
    //~ ROS_INFO_STREAM("int dx = " << (int)dx << " int dy = " << (int)dy << " dtheta = " << dtheta);
    _x += dx;
    _y += dy;
    _theta += dtheta;
    //~ ROS_INFO_STREAM("new x "<< _x << "  new y " << _y << " new theta " << _theta);
    //~ ROS_INFO_STREAM ("dt" << " " << dt.toSec() << " " << "linear" << " " << linear << " " << "angular" << " " << angular);
}

void Particle::sense(float angle, unsigned int width, unsigned int height, int** data, float resolution, int i)
{
	int distance = 1 ;
	float new_x = _x / resolution + distance * cos(angle);
	float new_y = _y / resolution + distance * sin(angle);
	
	while ( ((int)new_x < width && (int)new_y < height) && ((int)new_x >= 0 && (int)new_y >= 0))
	{
		//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
		if (data[(int)(new_x)][(int)(new_y)] > 50)
		{
			_particle_ranges[i] = (distance * resolution);
			return;
		}
		else
		{
			distance ++;
			new_x = _x / resolution + distance * cos(angle);
			new_y = _y / resolution + distance * sin(angle);
		}
	}
	_particle_ranges[i] = ( (distance-1) * resolution);
}

void Particle::setParticleWeight(unsigned int width, unsigned int height, int** data, float resolution, std::vector<float> ranges)
{
	std::vector<float> distances;
	float sum = 0;
	
	sense(PI + _theta, width, height, data, resolution, 0);
	sense(3 * PI/2 + _theta, width, height, data, resolution, 1);
	sense(_theta, width, height, data, resolution, 2);
	sense(PI/2 + _theta, width, height, data, resolution, 3);
		
	//~ ROS_INFO_STREAM("Particle_ranges_size" << " " <<_particle_ranges.size());
	for ( unsigned int i = 0 ; i < 4 ; i++ )
	{
		distances.push_back (fabs(ranges[i]-_particle_ranges[i]));
		sum += distances[i];
		//~ ROS_INFO_STREAM("Distances: ");
		//~ ROS_INFO_STREAM("i = " << i << " " << distances[i]);
	}
	
	_weight = pow(1/(sum + 1), 3);
	//~ ROS_INFO_STREAM("Weight: " << " " << _weight);
}

float Particle::getX()
{
	return _x;
}

float Particle::getY() 
{
	return _y;
}

float Particle::getWeight()
{
	return _weight;
}
