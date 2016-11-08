#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from rapp_robot_api import RappRobot 
from nao_handler.srv import *
import tf

import rospy
import sys
import time
import math

class TrackingAndMotion:
	def __init__(self):
		self.rh = RappRobot()
		self.pub = rospy.Publisher(rospy.get_param('joint_angles_topic'), JointAnglesWithSpeed, queue_size=1)
		self.publ = rospy.Publisher(rospy.get_param('velocities_topic'), Twist, queue_size=1)
		self.obj_position_pub = rospy.Publisher(rospy.get_param('object_position_topic'), Twist, queue_size=1)

		self.predator_hunt_pub = rospy.Publisher(rospy.get_param('predator_hunt_topic'), \
			Polygon, queue_size = 10)
		self.s = rospy.Service('set_behavior', SetBehavior, self.set_behavior)
		self.robot_state_service = rospy.Service('robot_state', RobotState, \
			self.setRobotState)
		self.rh.motion.enableMotors()
		
		self.lock_motion = False
		self.hunt_initiated = False
		self.find_distance_with_sonars = False
		self.state_flag = True
		self.x_vel = 0
		self.y_vel = 0
		self.theta_vel = 0
		self.robot_x = 0
		self.robot_y = 0
		self.dx = 0.0
		self.dy = 0.0
		self.path = []
		self.predator_topic = rospy.get_param('predator_topic')
		self.sonar_value = rospy.get_param('sonar_limit_value')
		self.head_pitch_value = rospy.get_param('head_pitch_limit_value')
		self.head_yaw_value = rospy.get_param('head_yaw_limit_value')
		
		
		self.head_motion_sampler_init = 4
		self.head_motion_sampler = self.head_motion_sampler_init

		self.set_vel_timer = rospy.Timer(rospy.Duration(0.1), self.set_velocities_callback)
		
		self.set_object_position_timer = rospy.Timer(rospy.Duration(0.1), self.set_object_position_callback)
		
		self.get_robot_pos_timer = rospy.Timer(rospy.Duration(0.1), self.get_robot_position_callback)

		self.obstacle_timer = rospy.Timer(rospy.Duration(0.1), self.obstacle_avoidance_callback)
		self.obstacle_timer.shutdown()
		self.lost_obj_timer = rospy.Timer(rospy.Duration(0.1), self.lost_object_callback)
		self.lost_obj_timer.shutdown()
		
		self.head_motion_timer = rospy.Timer(rospy.Duration(1), self.headMotionCallback)
		self.head_motion_timer.shutdown()
		
		self.object_tracking_sub = rospy.Subscriber(self.predator_topic, Polygon, self.track_bounding_box)
		self.object_tracking_sub.unregister()
		
		self.head_motion = False

		self.listener = tf.TransformListener()
		self.rh.humanoid_motion.goToPosture("Stand", 0.7)

	
	def enableObstacleAvoidance(self):
		#~ self.rh.humanoid_motion.goToPosture("Stand", 0.7)
		self.obstacle_timer = rospy.Timer(rospy.Duration(0.5), self.obstacle_avoidance_callback)

	def disableObstacleAvoidance(self):
		self.obstacle_timer.shutdown()
		self.x_vel = 0
		self.y_vel = 0
		self.theta_vel = 0
	
	def enableObjectTracking(self):
		#~ self.rh.humanoid_motion.goToPosture("Stand", 0.7)
		self.object_tracking_sub = rospy.Subscriber(self.predator_topic, Polygon, self.track_bounding_box)
		self.lost_obj_timer = rospy.Timer(rospy.Duration(0.1), self.lost_object_callback)
		self.lost_object_counter = 50
		self.lock_motion = False
		self.hunt_initiated = False
	
	def disableObjectTracking(self):
		self.lost_obj_timer.shutdown()
		self.object_tracking_sub.unregister()
	
	def track_bounding_box(self, polygon):
		
		self.state_flag = True
		self.hunt_initiated = True
		self.lost_object_counter = 50
		
		joint = JointAnglesWithSpeed()
	
		joint.joint_names.append("HeadYaw")
		joint.joint_names.append("HeadPitch")

		
		joint.speed = 0.15
		joint.relative = True

		target_x = polygon.points[0].x + 0.5 * polygon.points[1].x
		target_y = polygon.points[0].y + 0.5 * polygon.points[1].y
		
		sub_x = target_x - 320 / 2.0
		sub_y = target_y - 240 / 2.0
 
		var_x = (sub_x / 160.0)
		var_y = (sub_y / 120.0)
		
		#~ print 'var ', var_x, var_y
		
		joint.joint_angles.append(-var_x * 0.05)
		joint.joint_angles.append(var_y * 0.05)
				
		#~ print self.rh.sensors.getSonarsMeasurements()
		
		ans = self.rh.humanoid_motion.getJointAngles(['HeadYaw', 'HeadPitch'])['angles']
		head_yaw = ans[0]
		head_pitch = ans[1]
		
		
		sonars = self.rh.sensors.getSonarsMeasurements()['sonars']
		
		if (sonars['front_left'] <= self.sonar_value or sonars['front_right'] <= self.sonar_value) and self.lock_motion == False:
			self.lock_motion = True
			rospy.loginfo("Locked due to sonars")
			self.find_distance_with_sonars = True
			
		elif (head_pitch >= self.head_pitch_value or head_pitch <= -self.head_pitch_value) and self.lock_motion == False:
			self.lock_motion = True
			rospy.loginfo("Locked due to head pitch")
			
		print "self.lock_motion:", self.lock_motion
		
		if self.lock_motion is False:
			
			#~ self.theta_vel = 0
			if -self.head_yaw_value < head_yaw  and head_yaw < self.head_yaw_value:
				#~ self.x_vel = 0
				self.x_vel = 0.2
				self.theta_vel = 0
			else:
				self.x_vel = 0.0001
				self.theta_vel = head_yaw * 0.1
			
			self.head_motion_sampler -= 1
			if self.head_motion_sampler == 0:
				self.pub.publish(joint)
				self.head_motion_sampler = self.head_motion_sampler_init
		else:
			self.x_vel = 0
			self.y_vel = 0
			self.theta_vel = 0
			
		if  self.lock_motion is True:
			#~ self.disableObjectTracking()
			
			
			#~ print "sonars:", sonars['front_left'], sonars['front_right']
			#~ print "Head_pitch:",head_pitch
			#~ print "Head_yaw:",head_yaw
			#~ print "Sub_x:", sub_x
			#~ print "Sub_y:", sub_y
			#~ 
			#~ dx = 0
			sy = 0
			if self.find_distance_with_sonars is True and\
				(sonars['front_left'] <= self.sonar_value or sonars['front_right'] <= self.sonar_value):				
				if (sonars['front_left'] <= sonars['front_right']):
					self.dx = sonars['front_left']
					sy = +1
				else:
					self.dx = sonars['front_right']
					sy = -1
			else:
				x = (sub_y * 47.6* 3.14159 / 180) / 240.0
				#~ print "x=", x
				total_x = head_pitch + x + 0.021
				#~ print "total_x=",total_x
				self.dx = 0.53 / math.tan(total_x)
				sy = -1
				
			#~ print "dx= ",self.dx
			y = (sub_x * 60.9 * 3.14159 / 180) / 320.0
			#~ print "y=", y
			total_y = head_yaw + sy * y
			#~ print "total_y=",total_y
			self.dy = self.dx * math.tan(total_y)
			#~ print "dy= " ,self.dy
			
			self.disableObjectTracking()
			
			
		battery = self.rh.sensors.getBatteryLevels()['levels'][0]
		
		if battery < 25:
			self.rh.audio.setVolume(100)
			self.rh.audio.speak("My battery is low")
			self.sub.unregister()
			self.rh.humanoid_motion.goToPosture("Sit", 0.7)
			self.rh.motion.disableMotors()
			sys.exit(1)
			
	def set_object_position_callback(self, event):
		object_pos = Twist()
		
		object_pos.linear.x = self.dx
		object_pos.linear.y = self.dy
		#~ print object_pos
		
		self.obj_position_pub.publish(object_pos)
		
		self.dx = 0.0
		self.dy = 0.0
	
	def lost_object_callback(self, event):
		if self.hunt_initiated:
			self.lost_object_counter -= 1
		if self.lost_object_counter < 0 and self.hunt_initiated == True:
			rospy.loginfo("Locked due to 2 seconds")
			self.lock_motion = True
			self.x_vel = 0.0
			self.y_vel = 0.0
			self.theta_vel = 0.0
			
			self.disableObjectTracking()
			
	def set_velocities_callback(self, event):
		
		if self.state_flag == False:
			
			self.x_vel = 0.0
			self.y_vel = 0.0
			self.theta_vel = 0.0

		self.rh.motion.moveByVelocity(self.x_vel, self.y_vel, self.theta_vel)
		#~ print self.x_vel, self.theta_vel
		
		
		velocities = Twist()
		
		r = self.rh.motion.getVelocities()['velocities']
		
		velocities.linear.x = r[0]
		velocities.linear.y = r[1]
		velocities.angular.z = r[2]
		
		self.publ.publish(velocities)
		#~ print velocities
		
		
	def headMotionCallback(self, event):
		pass
		#~ if self.head_motion == False:
			#~ print "head motion changed False"
			#~ self.head_motion = True
			#~ self.rh.humanoid_motion.setJointAngles(["HeadYaw"],[0.2], 0.1)
		#~ else:
			#~ print "head motion changed True"
			#~ self.head_motion = False
			#~ self.rh.humanoid_motion.setJointAngles(["HeadYaw"],[-0.4], 0.1)
		
	def obstacle_avoidance_callback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()['sonars']
		
		if sonars['front_left'] <= self.sonar_value:
			self.x_vel = 0.0
			self.theta_vel = -(0.8 - sonars['front_left'])
		elif sonars['front_right'] <= self.sonar_value:
			self.x_vel = 0.0
			self.theta_vel = 0.8 - sonars['front_right']
		else:
			self.x_vel = 0.5
			self.theta_vel = 0.0
		
	
	def set_behavior(self, request):
		print 'Going to behavior: ' + request.behavior
		if request.behavior =="track_bounding_box":
			#~ self.rh.audio.speak("Tracking service")
			self.disableObstacleAvoidance()
			print 'Obstacle avoidance disabled'
			self.predator_hunt_pub.publish(request.polygon)
			self.enableObjectTracking()
			print 'Object tracking enabled'
			
		elif request.behavior =="obstacle_avoidance":
			#~ self.rh.audio.speak("Obstacle avoidance service")
			self.disableObjectTracking()
			print 'Object tracking disabled'
			self.enableObstacleAvoidance()
			print 'Obstacle avoidance enabled'
			self.move_head_timer = rospy.Timer(rospy.Duration(5), self.headMotionCallback)
			print 'Head motion enabled'
		
		print 'Behavior set'
		res = SetBehavior()
		res.success = True
		return True
		
	def setRobotState(self, request):
		if request.state <> self.state_flag:
			self.state_flag = request.state
		
		print 'Robot state set'
		print self.state_flag
		res = RobotState()
		res.success = True
		return True
		
	def get_robot_position_callback(self, event):
		#~ pass
		try:
			(trans,rot) = self.listener.lookupTransform('/map', '/nao_pose', rospy.Time(0))
			self.robot_x = trans[0]
			self.robot_y = trans[1]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
			#~ print ex
			pass
				
		
if __name__ == "__main__":
	rospy.init_node('nao_motion', anonymous=True)
	nao = TrackingAndMotion()
	rospy.spin()	
