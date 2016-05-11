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
from ogmpp_communications.srv import OgmppPathPlanningSrv
import tf

import rospy
import sys
import time

class MoveHeadAndBody:
	def __init__(self):
		self.rh = RappRobot()
		self.pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)
		self.publ = rospy.Publisher('/inner/cmd_vel', Twist, queue_size=1)
		self.path_publisher = rospy.Publisher('/as/path', \
			Path, queue_size = 10)
		self.s = rospy.Service('set_behavior', SetBehavior, self.set_behavior)
		self.service_path = rospy.Service('get_path', GetPath, self.get_path)
		self.rh.motion.enableMotors()
		self.rh.humanoid_motion.goToPosture("Stand", 0.7)
		self.lost_object_counter = 20
		self.lock_motion = False
		self.hunt_initiated = False
		self.x_vel = 0
		self.y_vel = 0
		self.theta_vel = 0
		self.robot_x = 0
		self.robot_y = 0
		self.path = []
		rospy.Timer(rospy.Duration(0.1), self.lost_object_callback)
		rospy.Timer(rospy.Duration(0.1), self.set_velocities_callback)
		rospy.Timer(rospy.Duration(0.1), self.get_robot_position_callback)
	
	def track_bounding_box(self, polygon):
		self.hunt_initiated = True
		self.lost_object_counter = 20
		
		joint = JointAnglesWithSpeed()
	
		joint.joint_names.append("HeadYaw")
		joint.joint_names.append("HeadPitch")

		#~ print polygon
		joint.speed = 0.1
		joint.relative = True

		target_x = polygon.points[0].x + 0.5 * polygon.points[1].x
		target_y = polygon.points[0].y + 0.5 * polygon.points[1].y
		
		sub_x = target_x - 320 / 2.0
		sub_y = target_y - 240 / 2.0
 
		var_x = (sub_x / 160.0)
		var_y = (sub_y / 120.0)
		
		joint.joint_angles.append(-var_x * 0.05)
		joint.joint_angles.append(var_y * 0.05)
				
		print self.rh.sensors.getSonarsMeasurements()
		
		[ans, err] = self.rh.humanoid_motion.getJointAngles(['HeadYaw', 'HeadPitch'])
		head_yaw = ans[0]
		head_pitch = ans[1]
		#~ print 'HeadPitch' + str(head_pitch)
		#~ print 'HeadYaw' + str(head_yaw)
		
		
		sonars = self.rh.sensors.getSonarsMeasurements()[0]
		
		if sonars['front_left'] <= 0.3 or sonars['front_right'] <= 0.3:
			self.lock_motion = True
			rospy.loginfo("Locked due to sonars")
		elif head_pitch >= 0.4 or head_pitch <= -0.4:
			self.lock_motion = True
			rospy.loginfo("Locked due to head pitch")
		else:
			if self.lock_motion is False:
				self.theta_vel = head_yaw * 0.1
				if -0.2 < head_yaw < 0.2:
					self.x_vel = 0.3
				self.pub.publish(joint)
			else:
				self.x_vel = 0
				self.y_vel = 0
				self.theta_vel = 0
				self.sub.unregister()
			
		[batt, none] = self.rh.sensors.getBatteryLevels()
		battery = batt[0]
		
		if battery < 25:
			self.rh.audio.setVolume(100)
			self.rh.audio.speak("My battery is low")
			self.sub.unregister()
			self.rh.humanoid_motion.goToPosture("Sit", 0.7)
			self.rh.motion.disableMotors()
			sys.exit(1)
			
	def lost_object_callback(self, event):
		if self.hunt_initiated:
			self.lost_object_counter -= 1
		if self.lost_object_counter < 0:
			self.lock_motion = True
			self.x_vel = 0.0
			self.y_vel = 0.0
			self.theta_vel = 0.0
			rospy.loginfo("Locked due to 2 seconds")
			self.sub.unregister()
			
			
	def set_velocities_callback(self, event):
		
		self.rh.motion.moveByVelocity(self.x_vel, self.y_vel, self.theta_vel)
		
		
		velocities = Twist()
		
		[r,e] = self.rh.motion.getVelocities()
		
		velocities.linear.x = r[0]
		velocities.linear.y = r[1]
		velocities.angular.z = r[2]
		
		#~ rospy.loginfo("%s", velocities)
		self.publ.publish(velocities)
		
		
	def obstacle_avoidance_callback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()[0]
		
		if sonars['front_left'] <= 0.5:
			self.x_vel = 0.0
			self.theta_vel = -(0.8 - sonars['front_left'])
		elif sonars['front_right'] <= 0.5:
			self.x_vel = 0.0
			self.theta_vel = 0.8 - sonars['front_right']
		else:
			self.x_vel = 0.5
			self.theta_vel = 0.0		
		
		self.rh.motion.moveByVelocity(self.x_vel, self.y_vel, self.theta_vel)
	
	def set_behavior(self, request):
		if request.behavior =="track_bounding_box":
			self.sub = rospy.Subscriber("/vision/predator_alert", Polygon, self.track_bounding_box)
			
			obstacle_timer = rospy.Timer(rospy.Duration(0.1), self.obstacle_avoidance_callback)
			obstacle_timer.shutdown()
			
			res = SetBehavior()
			res.success = True
			return True
		elif request.behavior =="obstacle_avoidance":
			obstacle_timer = rospy.Timer(rospy.Duration(0.1), self.obstacle_avoidance_callback)
			
			self.sub = rospy.Subscriber("/vision/predator_alert", Polygon, self.track_bounding_box)
			self.sub.unregister()			
			res = SetBehavior()
			res.success = True
			return True
			
	def get_path(self, request):
		rospy.wait_for_service('/ogmpp_path_planners/plan')
		try:
			path = rospy.ServiceProxy('/ogmpp_path_planners/plan', OgmppPathPlanningSrv)
			path.method = "uniform_prm"
			path.data.begin.x = self.robot_x
			path.data.begin.y = self.robot_y
			path.data.end.x = request.x
			path.data.end.y = request.y
			
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		ros_path = Path()
		
		#~ ros_path.header.frame_id = "map"
		_map = OccupancyGrid()
        
		for p in self.path:
			ps = PoseStamped()
			ps.header.frame_id = "map"
			ps.pose.position.x = p[0] * _map.info.resolution + \
				_map.info.origin['x']
			ps.pose.position.y = p[1] * _map.info.resolution + \
				_map.info.origin['y']
			
			ros_path.poses.append(ps)
		
		self.path_publisher.publish(ros_path)
		
				
	def get_robot_position_callback(self, event):
		robot = 
		if robot.ns == "Best Particle":
			self.robot_x = robot.points.x
			self.robot_y = robot.points.y
		
		rospy.loginfo("%s",robot)
		rospy.loginfo("robot ns:%s",robot.ns)
		rospy.loginfo("robot x:%s",self.robot_x)
		rospy.loginfo("robot y:%s",self.robot_y)
			
		
if __name__ == "__main__":
	rospy.init_node('nao_head', anonymous=True)
	nao = MoveHeadAndBody()
	rospy.spin()	

		
