#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Twist
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
		self.publ = rospy.Publisher('/inner/cmd_vel', Twist, queue_size=1)
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
		print 'HeadPitch' + str(head_pitch)
		print 'HeadYaw' + str(head_yaw)
		
		x_vel = 0
		y_vel = 0
		theta_vel = 0
		
		sonars = self.rh.sensors.getSonarsMeasurements()[0]
		
		if sonars['front_left'] <= 0.3 or sonars['front_right'] <= 0.3:
			self.lock_motion = True
			print 'Locked due to sonars'
		elif head_pitch >= 0.4 or head_pitch <= -0.4:
			self.lock_motion = True
			print 'Locked due to head pitch'
		else:
			theta_vel = head_yaw * 0.1
			if -0.2 < head_yaw < 0.2:
				x_vel = 0.3
		
		print x_vel, y_vel, theta_vel
			
		if self.lock_motion is False:
			self.set_velocities(x_vel, y_vel, theta_vel)
			self.pub.publish(joint)
		else:
			self.set_velocities(0, 0, 0)
			
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
			self.rh.motion.moveByVelocity(0, 0, 0)
			print 'Locked due to 2 seconds'
			
	def set_velocities(self, x, y, theta):
		velocities = Twist()
		
		self.rh.motion.moveByVelocity(x, y, theta)
		[r,e] = self.rh.motion.getVelocities()
		
		# r is in [-1,1] where 1 is the max speed
		# you must put m/s and rad/sec in velocities
		
		velocities.linear.x = r[0]
		velocities.linear.y = r[1]
		velocities.angular.z = r[2]
		
		print x,y,theta,str(r)
		self.publ.publish(velocities)
		
		
if __name__ == "__main__":
	rospy.init_node('nao_head', anonymous=True)
	nao = MoveHeadAndBody()
	rospy.spin()	

		
