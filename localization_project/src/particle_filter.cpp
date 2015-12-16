#include "particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	_linear = 0;
	_angular = 0;
	_particles_initialized = false;
	
	_particle_initialization_service = _n.advertiseService (
		"particle_initialization",
		&ParticleFilter::particlesInit, this);
	ROS_INFO("Service ready to initialize particles");
	
	if(!_n.getParam("/enable_visualization", _visualization_enabled))
    {
		ROS_ERROR("Enable_visualization param does not exist");
	}
	_visualization_pub = _n.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
            
    _velocity_sub = _n.subscribe("/robot0/cmd_vel", 5, &ParticleFilter::velocityCallback, this);
}

bool ParticleFilter::particlesInit (
	localization_project::particleInitSrv::Request& req,
	localization_project::particleInitSrv::Response& res
	)
{
	if(_n.getParam("/particles_number", _particles_number))
	{
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
		
		ros::Time time;
		ros::Duration dt;
			
		while (_particles_initialized)
		{
			dt = ros::Time::now() - time;
			
			for (unsigned int i = 0 ; i < _particles_number ; i++ )
			{
				_particles[i].move(dt, _linear, _angular);
				
				_particles[i].setParticleWeight(robot_percept.getMapWidth(),
					robot_percept.getMapHeight(), robot_percept.getMapData(),
					robot_percept.getMapResolution(),
					robot_percept.getLaserRanges());
				ROS_INFO_STREAM("linear = " << " " << _linear << "angular = " << " " << _angular);

				resample();
			}
		}	
			
		res.success = true;
		return true;
	}
	else
	{
		ROS_ERROR("Particles number param does not exist");
		res.success = false;
		return false;
	}
	
	//~ ROS_INFO_STREAM ("Particles:");
	//~ ROS_INFO_STREAM ("ParticleFilter:map_width ="<< " " << robot_percept.getMapWidth() << " " << "ParticleFilter:map_height ="<< " " << robot_percept.getMapHeight());

	//~ for (unsigned int i = 0 ; i < robot_percept.getMapWidth() ; i++ )
	//~ {
		//~ for (unsigned int j = 0 ; j < robot_percept.getMapHeight() ; j++ ) 
		//~ {
			//~ ROS_INFO_STREAM (" i = " << " " << i << " " << "j = " << " " << j << " " <<" Map data = " << " " << robot_percept.getMapCell(i,j));
		//~ }
	//~ }
}

//~ void ParticleFilter::particlesCallback()
//~ {	
	//~ ros::Time time;
	//~ ros::Duration dt;
		//~ 
	//~ while (_particles_initialized)
	//~ {
		//~ dt = ros::Time::now() - time;
		//~ 
		//~ for (unsigned int i = 0 ; i < _particles_number ; i++ )
		//~ {
			//~ _particles[i].move(dt, _linear, _angular);
			//~ 
			//~ _particles[i].setParticleWeight(robot_percept.getMapWidth(),
				//~ robot_percept.getMapHeight(), robot_percept.getMapData(),
				//~ robot_percept.getMapResolution(),
				//~ robot_percept.getLaserRanges());
			//~ ROS_INFO_STREAM("linear = " << " " << _linear << "angular = " << " " << _angular);
//~ 
			//~ resample();
			//~ 
//~ 
		//~ }
	//~ }
//~ }

void ParticleFilter::resample()
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
	_particles = new_particles;
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
    
    for(unsigned int i = 0 ; i < _particles_number ; i++ )
    {
      geometry_msgs::Point p;
      p.x = _particles[i].getX() * resolution;
      p.y = _particles[i].getY() * resolution;
      m.points.push_back(p);
    }
	
	 _visualization_pub.publish(m);
}

void ParticleFilter::velocityCallback(geometry_msgs::Twist twist)
{
	_linear = twist.linear.x;
	_angular = twist.angular.z;
}
