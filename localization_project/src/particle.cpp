#include "particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width, unsigned int height, int** data, std::vector<float> ranges, float resolution) 
{	
	_x = std::rand() % (width) * resolution;
	_y = std::rand() % (height) * resolution;
	 
	while ((data[(int)(_x / resolution)][(int)(_y / resolution)] == - 1)
		|| (data[(int)(_x / resolution)][(int)(_y / resolution)] > 50))
	{
		_x = std::rand() % (width) * resolution;
		_y = std::rand() % (height) * resolution;
	}

	_theta = static_cast <float> (rand()) / static_cast <float> 
		(RAND_MAX/2*PI);
	_dx = _dy = _dtheta = _i = 0;
	//~ _theta = 0;
	//~ ROS_INFO_STREAM("theta = " << _theta);

		
	_particle_ranges = new float[ranges.size()];	
	//~ ROS_INFO_STREAM ("Initial x = "<< " " << _x << " " << "y = "<< _y << " " << " theta = " << " " << _theta);
	
	//~ _x = 15;
	//~ _y = 15;
	//~ _theta = 0.18;
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

void Particle::move()
{
    ROS_INFO_STREAM("Move: dx = " << _dx << " dy = " << _dy << " dtheta = " << _dtheta );
    _x += _dx;
    _y += _dy;
    _theta += _dtheta;
    ROS_INFO_STREAM("x = " << _x << " y = " << _y << " theta = " << _theta);
    _dx = _dy = _dtheta = _i = 0;
    //~ ROS_INFO_STREAM("new x "<< _x << "  new y " << _y << " new theta " << _theta);
    //~ ROS_INFO_STREAM ("dt" << " " << dt.toSec() << " " << "linear" << " " << linear << " " << "angular" << " " << angular);
}

void Particle::sense(float angle, unsigned int width, unsigned int height, int** data, float resolution, int i)
{
	int distance = 1;
	float new_x = _x / resolution + distance * cos(angle);
	float new_y = _y / resolution + distance * sin(angle);
	
	while (((int)new_x < width && (int)new_y < height) && ((int)new_x >= 0 && (int)new_y >= 0))
	{
		//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
		if ((data[(int)(new_x)][(int)(new_y)] > 50) ||  (data[(int)(new_x)][(int)(new_y)] == -1))
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

void Particle::calculateMotion(float previous_linear, float previous_angular, ros::Duration dt, float deviation)
{
	if (previous_angular == 0)
	{
	  _dx += (previous_linear * dt.toSec() * cosf(_theta));
	  _dy += (previous_linear * dt.toSec() * sinf(_theta));
	}
	else
	{
	  _dx += (- previous_linear / previous_angular * sinf(_theta)
		+ previous_linear / previous_angular * 
		sinf(_theta + dt.toSec() * previous_angular));
	  
	  _dy -= (- previous_linear / previous_angular * cosf(_theta)
		+ previous_linear / previous_angular * 
		cosf(_theta + dt.toSec() * previous_angular));
	}
	
	_dtheta += previous_angular * dt.toSec();
	
	ROS_INFO_STREAM("CalculateMotion:");
	ROS_INFO_STREAM("counter = " << _i << " linear = " << previous_linear
		<< " angular = " << previous_angular << " dt = " << dt << " dx = "
		<< _dx << " dy = " << _dy << " dtheta = " << _dtheta );
	_i++;
	//~ _linear = previous_linear + _noise_param1 * fabs(previous_linear) +
			//~ _noise_param2 * fabs(previous_angular);
	//~ _angular = previous_angular + _noise_param1 * fabs(previous_linear) +
			//~ _noise_param2 * fabs(previous_angular);
	//~ _x1 = _x - (_linear / _angular * sinf(_theta) +
		//~ _linear / _angular * sinf(_theta +
			//~ _angular * _dt.toSec())) / robot_percept.getMapResolution();
	//~ _y1 = _y + (_linear / _angular * cosf(_theta) -
		//~ _linear / _angular * cosf(_theta +
			//~ _angular * _dt.toSec())) / robot_percept.getMapResolution(); 
	//~ _theta1 = _theta + _angular * _dt.toSec() + (noise() +
		//~ _noise_param1 * fabs(_previous_linear) +
			//~ _noise_param2 * fabs(_previous_angular)) * _dt.toSec();
	
	//~ ROS_INFO_STREAM("Linear : " << previous_linear << " linear_noise : " << _linear);
	//~ ROS_INFO_STREAM("Angular : " << previous_angular << " angular_noise : " << _angular);
	//~ ROS_INFO_STREAM("x : " << _x << " x1 : " << _x1);
	//~ ROS_INFO_STREAM("y : " << _y << " y1 : " << _y1);
	//~ ROS_INFO_STREAM("theta : " << _theta << " theta1 : " << _theta1);

	//~ ROS_INFO_STREAM ( "angular = " << " " << _current_angular << " " << "linear=" << " " << _current_linear);
	//~ ROS_INFO_STREAM ( "dx = " << _x << " " << " dy = " << _y << " dtheta = " << _theta);
	//~ ROS_INFO_STREAM ( "dt = " << " " << _dt.toSec());
}

float Particle::noise(float deviation) 
{
	float sum = 0;
	for ( unsigned int i = 0 ; i < 12 ; i++)
	{
		sum += - deviation + static_cast <float>
			(rand()) /( static_cast <float> (RAND_MAX/(2 * deviation)));
		//~ ROS_INFO_STREAM("function noise i = " << i << " noise = " << -deviation + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*deviation))));
	}
	return 0.5*sum;
}

void Particle::setParticleWeight(unsigned int width, unsigned int height, int** data, float resolution, std::vector<float> ranges, float max_range)
{
	std::vector<float> distances;
	float sum = 0;
	
	sense(PI + _theta, width, height, data, resolution, 0);
	sense(3 * PI/2 + _theta, width, height, data, resolution, 1);
	sense(_theta, width, height, data, resolution, 2);
	sense(PI/2 + _theta, width, height, data, resolution, 3);

	for ( unsigned int i = 0 ; i < 4 ; i++ )
	{
		ROS_INFO_STREAM("particle_ranges : " << _particle_ranges[i]);
		if (_particle_ranges[i] / resolution <= 1)
		{
			_weight = 0;
			return;
		}
		else
		{	
			if (_particle_ranges[i] > max_range)
				_particle_ranges[i] = max_range;
			distances.push_back (fabs(ranges[i]-_particle_ranges[i]));
			sum += distances[i];
		}
	}
	
	//~ _weight = pow(1/(sum + 1), 2);
	_weight = 1 / (sum + 1);
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
