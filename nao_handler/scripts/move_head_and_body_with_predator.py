#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import CameraInfo
from rapp_robot_api import RappRobot

import rospy
import sys
import time

class MoveHeadAndBody:
	def __init__(self):
		self.rh = RappRobot()
		self.sub = rospy.Subscriber("/vision/predator_alert", Polygon, self.move)
		self.pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=1)
		self.rh.motion.enableMotors()
		self.rh.humanoid_motion.goToPosture("Stand", 0.7)
		self.lost_object_counter = 20
		self.lock_motion = False
		self.hunt_initiated = False
		rospy.Timer(rospy.Duration(0.1), self.lost_object_callback)
	
	def move(self, polygon):
		self.hunt_initiated = True
		self.lost_object_counter = 20
		
		joint = JointAnglesWithSpeed()
		
		#~ image = CameraInfo()
		#~ image = CameraInfo()
		#~ 
		#~ print image.height
		#~ print image.width
		

		joint.joint_names.append("HeadYaw")
		joint.joint_names.append("HeadPitch")

		j = joint.joint_angles
		#~ print polygon
		joint.speed = 0.01
		joint.relative = True

		target_x = polygon.points[0].x + 0.5 * polygon.points[1].x
		target_y = polygon.points[0].y + 0.5 * polygon.points[1].y
		
		sub_x = target_x - 320 / 2.0
		sub_y = target_y - 240 / 2.0
 
		var_x = (sub_x / 160.0)
		var_y = (sub_y / 120.0)
		
		joint.joint_angles.append(-var_x * 0.05)
		joint.joint_angles.append(var_y * 0.05)
				
		#~ print rh.sensors.getSonarsMeasurements()
		
		[ans, err] = self.rh.humanoid_motion.getJointAngles(['HeadYaw', 'HeadPitch'])
		head_yaw = ans[0]
		head_pitch = ans[1]
		#~ print 'HeadPitch' + str(head_pitch)
		
		x_vel = 0
		y_vel = 0
		theta_vel = 0
		
		if self.rh.sensors.getSonarsMeasurements()[0]['front_left'] <= 0.3:
			self.lock_motion = True
			print 'Locked due to sonars'
		elif head_pitch >= 0.4 or head_pitch <= -0.4:
			self.lock_motion = True
			print 'Locked due to head pitch'
		else:
			theta_vel = head_yaw * 0.1
			if head_yaw < 0.02 or head_yaw > -0.02:
				x_vel = 0.3
				y_vel = 0.3
			
		if self.lock_motion is False:
			self.rh.motion.moveByVelocity(x_vel, y_vel, theta_vel)
			self.pub.publish(joint)
		else:
			self.rh.motion.moveByVelocity(0, 0, 0)
			
	def lost_object_callback(self, event):
		if self.hunt_initiated:
			self.lost_object_counter -= 1
		if self.lost_object_counter < 0:
			self.lock_motion = True
			self.rh.motion.moveByVelocity(0, 0, 0)
			print 'Locked due to 2 seconds'
		
if __name__ == "__main__":
	rospy.init_node('nao_head', anonymous=True)
	
	nao = MoveHeadAndBody()
	rospy.spin()	

		
