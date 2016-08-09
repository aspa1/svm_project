#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "robot_perception.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "localization_project/particleInitSrv.h"


class ParticleFilter {
	private:
		ros::NodeHandle _n;
		ros::ServiceServer _particle_initialization_service;
		ros::Publisher _visualization_pub;
		ros::Subscriber _velocity_sub;
		ros::Time _current_time;
		ros::Time _previous_time;
		ros::Timer _timer;
		ros::Duration _dt;
		std::string _velocity_topic;
		std::string inner_pose;
		RobotPerception robot_percept;
		int _particles_number;
		int _duration;
		int _sampling_step;
		float _noise_param1;
		float _noise_param2;
		float _strictness_param;
		float _current_linear;
		float _current_angular;
		float _previous_linear;
		float _previous_angular;
		float _initial_x;
		float _initial_y;
		float _initial_theta;
		std::vector<Particle> _particles; 
		bool _initial_pose_flag;
		bool _visualization_enabled;
		bool _particles_initialized;
		bool _flag;
		bool _motion_flag;
	

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
		
		
};

#endif
