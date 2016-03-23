#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "robot_perception.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include "localization_project/particleInitSrv.h"


class ParticleFilter {
	private:
		ros::NodeHandle _n;
		RobotPerception robot_percept;
		tf::TransformListener _listener;
		ros::ServiceServer _particle_initialization_service;
		int _particles_number;
		std::vector<Particle> _particles; 
		bool _visualization_enabled;
		ros::Publisher _visualization_pub;
		ros::Subscriber _velocity_sub;
		ros::Time _current_time;
		ros::Time _previous_time;
		float _current_linear;
		float _current_angular;
		float _previous_linear;
		float _previous_angular;
		float _linear;
		float _angular;
		bool _particles_initialized;
		ros::Timer _timer;
		bool _flag;
		float _deviation;
		float _noise_param1;
		float _noise_param2;
		int _duration;
		
		ros::Duration _dt;
		float _x;
		float _y;
		float _theta;
		float _x1;
		float _y1;
		float _theta1;

	
	public:
		ParticleFilter();
		bool particlesInit ( 
			localization_project::particleInitSrv::Request& req,
			localization_project::particleInitSrv::Response& res
		);
		void visualize(float resolution);
		void particlesCallback(const ros::TimerEvent& event);
		void velocityCallback(geometry_msgs::Twist twist);
		void resample();
		void calculateMotion(bool flag);
		float noise();
};

#endif
