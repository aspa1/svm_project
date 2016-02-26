#include "particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	_previous_linear = 0;
	_previous_angular = 0;
	_current_angular = 0;
	_current_linear = 0;
	_previous_time = ros::Time::now();
	_flag = false;
	_particles_initialized = false;
	
	_particle_initialization_service = _n.advertiseService (
		"particles_initialization",
		&ParticleFilter::particlesInit, this);
	ROS_INFO("Service ready to initialize particles");
	
	if(!_n.getParam("/enable_visualization", _visualization_enabled))
    {
		ROS_ERROR("Enable_visualization param does not exist");
	}
	if(!_n.getParam("standard_deviation", _deviation))
	{
		ROS_ERROR("Standard_deviation param does not exist");
	}
	if(!_n.getParam("a1", _noise_param1))
	{
		ROS_ERROR("Noise param a1 does not exist");
	}
	if(!_n.getParam("a2", _noise_param2))
	{
		ROS_ERROR("Noise param a2 does not exist");
	}
	_visualization_pub = _n.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
            
    _velocity_sub = _n.subscribe("/robot0/cmd_vel", 10,
		&ParticleFilter::velocityCallback, this);
    
    _timer = _n.createTimer(ros::Duration(20),
		&ParticleFilter::particlesCallback, this);
    
    _x = 0;
    _y = 0;
    _theta = 0;
}

bool ParticleFilter::particlesInit (
	localization_project::particleInitSrv::Request& req,
	localization_project::particleInitSrv::Response& res
	)
{
	if(!_n.getParam("/particles_number", _particles_number))
	{
		ROS_ERROR("Particles number param does not exist");
	}
	ROS_INFO_STREAM ("Particles " << _particles_number);
	for (unsigned int i = 0 ; i < _particles_number ; i++ )
	{
		Particle particle( robot_percept.getMapWidth(),
			robot_percept.getMapHeight(), robot_percept.getMapData());
		_particles.push_back(particle);
		//~ ROS_INFO_STREAM("Particle" << " " << i+1 << ":");
	}
	_particles_initialized = true;
	
	ROS_INFO_STREAM(_particles_number << " " << "particles initialized");
	if(_visualization_enabled)
		visualize(robot_percept.getMapResolution());
	
	res.success = true;
	return true;	
}

void ParticleFilter::particlesCallback(const ros::TimerEvent& event)
{
	//~ int counter = 0 ;
	ROS_INFO_STREAM("AOUA");
	if (_particles_initialized)
	{
		for (unsigned int i = 0 ; i < _particles_number ; i++ )
		{
			//ROS_INFO_STREAM("ParticlesCallback " << " dx = " << _x << "dy = " << _y << " dtheta = " << _theta);
			//ROS_INFO_STREAM ("noise: " << _particles[i].noise(_deviation));
			_particles[i].move(_x1, _y1, _theta1, robot_percept.getMapResolution());
		}
		visualize(robot_percept.getMapResolution());
		ROS_INFO_STREAM("AOUA2");
		for (unsigned int i = 0 ; i < _particles_number ; i++ )
		{
			_particles[i].setParticleWeight(robot_percept.getMapWidth(),
				robot_percept.getMapHeight(), robot_percept.getMapData(),
				robot_percept.getMapResolution(),
				robot_percept.getLaserRanges());
			//~ ROS_INFO_STREAM("linear = " << " " << _linear << "angular = " << " " << _angular);
		//~ counter++;
		}
		ROS_INFO_STREAM("PW + " << _particles[0].getWeight());
		ROS_INFO_STREAM("AOUA3");
		//~ ROS_INFO_STREAM("counter" << " " << counter);
		if (_x != 0 || _y != 0 || _theta != 0)
			resample();
		ROS_INFO_STREAM("AOUA4");
		visualize(robot_percept.getMapResolution());
	}
	_x = _y = _theta = 0;
}


void ParticleFilter::resample()
{
	bool flag = false;
	for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
	{
		if (_particles[i].getWeight() > 0.00001)
		{
			flag = true;
			break;
		}
	}
	//~ ROS_INFO_STREAM ("Flag = " << flag);
	if (flag == true)
	{
		std::vector<Particle> new_particles;
		int index = std::rand() % ( _particles_number );
		float beta = 0.0;
		float max_weight = _particles[0].getWeight();
		for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
		{
			if (_particles[i].getWeight() > max_weight)
				max_weight = _particles[i].getWeight();
		}
		for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
		{
			beta += static_cast <float> (rand()) / static_cast <float> 
				(RAND_MAX/ 2*max_weight);
			while (beta > _particles[index].getWeight())
			{
				beta -= _particles[index].getWeight();
				index = (index + 1) % _particles_number;
			}
			new_particles.push_back(_particles[index]);
		}
		ROS_INFO_STREAM("particles size = " << " " << _particles.size() << "new particles size = " << " " << new_particles.size());
		_particles = new_particles;
	}
}

void ParticleFilter::velocityCallback(geometry_msgs::Twist twist)
{
	ROS_INFO_STREAM("VelocityCallback");
	_current_linear = twist.linear.x;
	_current_angular = twist.angular.z;
	
	if (_particles_initialized)
	{
		if ((_current_angular != _previous_angular) || (_current_linear != _previous_linear))
		{
			if (_flag == false)
			{
				_previous_time = ros::Time::now();
				_previous_angular = _current_angular;
				_previous_linear = _current_linear;
				_flag = true;
			}
			else
			{
				_current_time = ros::Time::now();
				_dt = _current_time - _previous_time;
				
				if (_previous_angular == 0)
				{
				  _x += (_previous_linear * _dt.toSec() * cosf(_theta)) / robot_percept.getMapResolution();
				  _y += (_previous_linear * _dt.toSec() * sinf(_theta)) / robot_percept.getMapResolution();
				}
				else
				{
				  _x += (- _previous_linear / _previous_angular * sinf(_theta) + _previous_linear / _previous_angular * 
					sinf(_theta + _dt.toSec() * _previous_angular)) / robot_percept.getMapResolution();
				  
				  _y -= (- _previous_linear / _previous_angular * cosf(_theta) + _previous_linear / _previous_angular * 
					cosf(_theta + _dt.toSec() * _previous_angular)) / robot_percept.getMapResolution();
				}
				
				_theta += _previous_angular * _dt.toSec();
				_linear = _previous_linear + noise() + _noise_param1 * fabs(_previous_linear) + _noise_param2 * fabs(_previous_angular);
				_angular = _previous_angular + noise() + _noise_param1 * fabs(_previous_linear) + _noise_param2 * fabs(_previous_angular);
				_x1 = _x - _linear / _angular * sinf(_theta) + _linear / _angular * sinf(_theta + _angular * _dt.toSec());
				_y1 = _y + _linear / _angular * cosf(_theta) - _linear / _angular * cosf(_theta + _angular * _dt.toSec()); 
				_theta1 = _theta + _angular * _dt.toSec() + (noise() + _noise_param1 * fabs(_previous_linear) + _noise_param2 * fabs(_previous_angular)) * _dt.toSec();
				ROS_INFO_STREAM("Linear : " << _previous_linear << " linear_noise : " << _linear);
				ROS_INFO_STREAM("Angular : " << _previous_angular << " angular_noise : " << _angular);
				ROS_INFO_STREAM("x : " << _x << " x1 : " << _x1);
				ROS_INFO_STREAM("y : " << _y << " y1 : " << _y1);
				ROS_INFO_STREAM("theta : " << _theta << " theta1 : " << _theta1);
				_previous_angular = _current_angular;
				_previous_linear = _current_linear;
				_previous_time = _current_time;
			}
		}
	}
	ROS_INFO_STREAM ( "angular = " << " " << _current_angular << " " << "linear=" << " " << _current_linear);
	ROS_INFO_STREAM ( "x = " << _x << " " << " y = " << _y << " theta = " << _theta);
	ROS_INFO_STREAM ( "dt = " << " " << _dt.toSec());
}

void ParticleFilter::visualize(float resolution)
{
	visualization_msgs::Marker m;
	
	m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.ns = "Particles";
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    
    for(unsigned int i = 0 ; i < _particles_number ; i++)
    {
      geometry_msgs::Point p;
      p.x = _particles[i].getX() * resolution;
      p.y = _particles[i].getY() * resolution;
      m.points.push_back(p);
    }
	
	 _visualization_pub.publish(m);
}

float ParticleFilter::noise() 
{
	float sum = 0;
	for ( unsigned int i = 0 ; i < 12 ; i++)
	{
		sum += -_deviation + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*_deviation)));
		//~ ROS_INFO_STREAM("function noise i = " << i << " noise = " << -deviation + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*deviation))));
	}
	return 0.5*sum;
}
