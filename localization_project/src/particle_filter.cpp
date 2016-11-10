#include "particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	_previous_linear = 0;
	_previous_angular = 0;
	_current_angular = 0;
	_current_linear = 0;
	_previous_time = ros::Time::now();
	_flag = false;
	_motion_flag = false;
	_particles_initialized = false;
	
	_particle_initialization_service = _n.advertiseService (
		"particles_initialization",
		&ParticleFilter::particlesInit, this);
	ROS_INFO("Service ready to initialize particles");
	
	if(!_n.getParam("/initial_pose_known", _initial_pose_flag))
    {
		ROS_ERROR("Initial_pose_known param does not exist");
	}
	if (_initial_pose_flag)
	{
		if(!_n.getParam("/initial_x", _initial_x))
		{
			ROS_ERROR("Initial_x does not exist");
		}
		if(!_n.getParam("/initial_y", _initial_y))
		{
			ROS_ERROR("Initial_y param does not exist");
		}
		if(!_n.getParam("/initial_theta", _initial_theta))
		{
			ROS_ERROR("Initial_theta param does not exist");
		}
	}
	if(!_n.getParam("/enable_visualization", _visualization_enabled))
    {
		ROS_ERROR("Enable_visualization param does not exist");
	}
	if(!_n.getParam("duration", _duration))
	{
		ROS_ERROR("Duration param does not exist");
	}
	if(!_n.getParam("a1", _noise_param1))
	{
		ROS_ERROR("Noise param a1 does not exist");
	}
	if(!_n.getParam("a2", _noise_param2))
	{
		ROS_ERROR("Noise param a2 does not exist");
	}
	if(!_n.getParam("/velocity_topic", _velocity_topic))
    {
		ROS_ERROR("Velocity_topic param does not exist");
	}
	if(!_n.getParam("/rays_subsampling_step", _sampling_step))
    {
		ROS_ERROR("Sampling_step param does not exist");
	}
	if(!_n.getParam("/strictness", _strictness_param))
    {
		ROS_ERROR("Strictness param does not exist");
	}
	
	_visualization_pub = _n.advertise<visualization_msgs::Marker>(
		"visualization_marker", 0);
		            
    _velocity_sub = _n.subscribe(_velocity_topic, 10,
		&ParticleFilter::velocityCallback, this);
    
    _timer = _n.createTimer(ros::Duration(_duration),
		&ParticleFilter::particlesCallback, this);
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
	for (unsigned int i = 0 ; i < _particles_number ; i++ )
	{
		Particle particle(robot_percept.getMapWidth(),
			robot_percept.getMapHeight(), robot_percept.getMapData(),
			robot_percept.getLaserRanges(), robot_percept.getMapResolution(),
			_sampling_step);
		if (_initial_pose_flag)
			particle.setPose(_initial_x, _initial_y, _initial_theta);
		else
			particle.randomize(robot_percept.getMapWidth(),
				robot_percept.getMapHeight(), robot_percept.getMapData(),
				robot_percept.getMapResolution());
		_particles.push_back(particle);
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
	ros::Time time1, time2;
	ros::Duration dt;
	if (_particles_initialized)
	{
		time1 = ros::Time::now();
		if (_previous_angular || _previous_linear)
			_motion_flag = true;
		_current_time = ros::Time::now();
		_dt = _current_time - _previous_time;
		for (unsigned int i = 0 ; i < _particles_number ; i++)
		{
			_particles[i].calculateMotion(_previous_linear,
				_previous_angular, _dt, _noise_param1, _noise_param2);
			_particles[i].move();
			_previous_angular = _current_angular;
			_previous_linear = _current_linear;
			_previous_time = _current_time;
		}
		visualize(robot_percept.getMapResolution());
		for (unsigned int i = 0 ; i < _particles_number ; i++)
		{
			_particles[i].setParticleWeight(robot_percept.getMapWidth(),
				robot_percept.getMapHeight(), robot_percept.getMapData(),
				robot_percept.getMapResolution(), robot_percept.getLaserRanges(),
				robot_percept.getRangeMax(), robot_percept.getAngleIncrement(),
				robot_percept.getAngleMin(), robot_percept.getRfidPose(),
				_sampling_step, _strictness_param);
		}
		if (_motion_flag || ( (_flag == 0) &&
			(_particles[0].rfidSense(robot_percept.getRfidPose()) != 0) ))
		{
			resample();
		}
		
		_motion_flag = false;
		visualize(robot_percept.getMapResolution());
		time2 = ros::Time::now();
		dt = time2 - time1;
		//~ ROS_INFO_STREAM("dt = " << dt.toSec());
	}
}


void ParticleFilter::resample()
{
	float average = 0;
	float sum = 0;
	bool flag = false;
	for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
	{
		if (_particles[i].getWeight() > 0.001)
		{
			flag = true;
			break;
		}
	}
	for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
	{
		sum += _particles[i].getWeight();
	}
	average = sum/ _particles_number;
	//~ ROS_INFO_STREAM("average1 = " << average);
	if (flag == true)
	{
		//~ ROS_INFO_STREAM("resample");
		sum = 0;
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
				(RAND_MAX/ (2*max_weight));
			while (beta > _particles[index].getWeight())
			{
				beta -= _particles[index].getWeight();
				index = (index + 1) % _particles_number;
			}
			new_particles.push_back(_particles[index]);
		}
	
		_particles = new_particles;
		for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
		{
			sum += _particles[i].getWeight();
		}
		average = sum/ _particles_number;
		//~ ROS_INFO_STREAM("average2 = " << average);
	}
	else
	{
		for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
		{
			_particles[i].randomize(robot_percept.getMapWidth(),
				robot_percept.getMapHeight(), robot_percept.getMapData(),
				robot_percept.getMapResolution());
		}
		ROS_ERROR_STREAM("Oops! Robot lost, particles randomized");
	}
}

void ParticleFilter::velocityCallback(geometry_msgs::Twist twist)
{
	//~ ROS_INFO_STREAM("VelocityCallback");
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
			_motion_flag = true;
			_current_time = ros::Time::now();
			_dt = _current_time - _previous_time;
			for (int i = 0 ; i < _particles_number; i ++)
			{
				_particles[i].calculateMotion(_previous_linear,
					_previous_angular, _dt, _noise_param1, _noise_param2);
			}
			_previous_angular = _current_angular;
			_previous_linear = _current_linear;
			_previous_time = _current_time;
			}
		}
	}
}

void ParticleFilter::visualize(float resolution)
{
	visualization_msgs::Marker m, m1;
	
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
      p.x = _particles[i].getX();
      p.y = _particles[i].getY();
      m.points.push_back(p);
    }
	 _visualization_pub.publish(m);
	 
	m1.header.frame_id = "map";
    m1.header.stamp = ros::Time();
    m1.type = visualization_msgs::Marker::SPHERE_LIST;
    m1.action = visualization_msgs::Marker::ADD;
    m1.id = 0;
    m1.ns = "Best Particle";
    m1.scale.x = 0.35;
    m1.scale.y = 0.35;
    m1.scale.z = 0.35;
    m1.color.a = 1.0;
    m1.color.r = 0.0;
    m1.color.g = 0.0;
    m1.color.b = 1.0;
    
    float max_weight = _particles[0].getWeight();
    int id = 0;
	for (unsigned int i = 0 ; i < _particles_number ; i++ ) 
	{
		if (_particles[i].getWeight() > max_weight)
		{
			max_weight = _particles[i].getWeight();
			id = i;
		}
	}
	
	geometry_msgs::Point p1;
	p1.x = _particles[id].getX();
	p1.y = _particles[id].getY();
	m1.points.push_back(p1);
	
	//~ ROS_INFO_STREAM("Best particle: x = " << _particles[id].getX() <<
		//~ " y = " << _particles[id].getY() << " theta = " << _particles[id].getTheta());
	 _visualization_pub.publish(m1);
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(_particles[id].getX(), _particles[id].getY(), 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, _particles[id].getTheta());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "nao_pose"));
}
