#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import CameraInfo

import rospy
import sys
import time

class MoveNaoHead:
	def __init__(self):
		rospy.Subscriber("/vision/predator_alert", Polygon, self.move)
		self.pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
		self.call_body_stifness_server_enable()
	
	def move(self, polygon):
		
		joint = JointAnglesWithSpeed()
		
		#~ image = CameraInfo()
		#~ 
		#~ print image.height
		#~ print image.width
		

		joint.joint_names.append("HeadYaw")
		joint.joint_names.append("HeadPitch")

		j = joint.joint_angles

		joint.speed = 0.01
		joint.relative = True

		target_x = polygon.points[0].x + 0.5 * polygon.points[1].x
		target_y = polygon.points[0].y + 0.5 * polygon.points[1].y
		
		sub_x = target_x - 320 / 2.0
		sub_y = target_y - 240 / 2.0

		var_x = abs((sub_x / 320) * 0.1)
		var_y = abs((sub_y / 240) * 0.1)
		
		if var_x < 0.05 and var_x > -0.05:
			if var_y < 0.05 and var_y > -0.05:
				if sub_x < 0.0:
					joint.joint_angles.append(var_x)
				else:
					joint.joint_angles.append(-var_x)
					
				if sub_y < 0.0:
					joint.joint_angles.append(-var_y)
				else:
					joint.joint_angles.append(var_y)
		
		self.pub.publish(joint)
		
		print 'Move to ' + str(joint.joint_angles)
		
		time.sleep(0.1)

	def call_body_stifness_server_enable(self):
		print "hello1"
		rospy.wait_for_service('/body_stiffness/enable')
		try:
			print "hello21"
			service_handle = rospy.ServiceProxy('/body_stiffness/enable', Empty)
			resp1 = service_handle()
			print "Body stiffness is enable"
		except rospy.ServiceException, e:
			print "Body stiffness service call failed: %s"%e
	
	def call_body_stifness_server_disable(self):
		print "hello2"
		rospy.wait_for_service('/body_stiffness/disable')
		try:
			print "hello51"
			service_handle2 = rospy.ServiceProxy('/body_stiffness/disable', Empty)
			resp2 = service_handle2()
			print "Body stiffness is disable"
		except rospy.ServiceException, e:
			print "Body stiffness service call failed: %s"%e
	
		
if __name__ == "__main__":
	rospy.init_node('nao_head', anonymous=True)
	nao = MoveNaoHead()
	rospy.spin()	

		
