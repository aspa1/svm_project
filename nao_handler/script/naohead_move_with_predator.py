#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import CameraInfo

import rospy
import sys

class move_nao_head:
	def __init__(self):
		s = rospy.Service('/body_stiffness/enable', Empty, self.call_body_stifness_server)
		rospy.Subscriber("/joint_angles", JointAnglesWithSpeed, self.move)
		self.pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
		print "hello31"
	
	def move(self):
		rospy.init_node('nao_head', anonymous=True)
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			joint= JointAnglesWithSpeed()
			pred_point= Polygon() 
			#~ joint.joint_names.append("HeadYaw")
			#~ joint.joint_names[1]= "HeadPitch"
			#~ joint.joint_angles= 0*len(pred_point.points)
			for i in range(len(pred_point.points)-1):
				print "hello41"	
				joint.joint_angles[i]= pred_point.points[i]
				print joint.joint_angles[i]
				#~ joint.joint_angles[2]= pred_point.points[2]
			joint.speed= 0.1
			
			#~ hello_str = "hello world %s" % rospy.get_time()
			#~ rospy.loginfo(hello_str)
			for i in range(len(pred_point.points)-1):
				self.pub.publish(joint.joint_angles[i])
			
	def call_body_stifness_server(self):
		print "hello1"
		rospy.wait_for_service('/body_stiffness/enable')
		try:
			print "hello21"
			service_handle = rospy.ServiceProxy('/body_stiffness/enable', Empty)
			resp1 = service_handle()
			print "Body stiffness is enable"
		except rospy.ServiceException, e:
			print "Body stiffness service call failed: %s"%e
	
		
if __name__ == "__main__":
	nao= move_nao_head()
	nao.call_body_stifness_server()
	#~ print "hello1"
	nao.move()		

		
