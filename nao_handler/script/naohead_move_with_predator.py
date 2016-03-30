#!/usr/bin/env python

from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from sensor_msgs.msg import CameraInfo

import rospy
import sys

class move_nao_head:
	def __init__(self):
		s1 = rospy.Service('/body_stiffness/enable', Empty, self.call_body_stifness_server_enable)
		#~ s2 = rospy.Service('/body_stiffness/disable', Empty, self.call_body_stifness_server_disable)
		rospy.Subscriber("/predator/hunt", Polygon, self.move)
		rospy.Subscriber("/joint_angles", JointAnglesWithSpeed, self.move)
		self.pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
		print "hello31"
	
	def move(self,Polygon):
		rospy.init_node('nao_head', anonymous=True)
		#~ rate = rospy.Rate(10) # 10hz
		#~ while not rospy.is_shutdown():
		joint= JointAnglesWithSpeed()
		pred_point= Polygon
		p= pred_point.points
		print len(p)
		joint.joint_names.append("HeadYaw")
		joint.joint_names.append("HeadPitch")
		print joint.joint_names
		#~ joint.joint_names= "HeadPitch,HeadPitch"
		#~ joint.joint_angles= 0*len(pred_point.points)
		print "aoua"
		j= joint.joint_angles
		print len(j)
		#~ for i in range(len(pred_point.points)-1):
			#~ print "hello41"	
			#~ joint.joint_angles.append(pred_point.points[i])
			#~ print joint.joint_angles[i]
		joint.speed= 0.1
		joint.joint_angles.append(1)
		joint.joint_angles.append(1)
		print joint.joint_angles
		#~ joint.joint_angles= 1
		
		
		
		rospy.loginfo(joint)
		self.pub.publish(JointAnglesWithSpeed)
		
		#~ hello_str = "hello world %s" % rospy.get_time()
		#~ rospy.loginfo(hello_str)
		#~ for i in range(len(pred_point.points)-1):
			#~ self.pub.publish(joint.joint_angles[i])
			
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
	
	#~ def call_body_stifness_server_disable(self):
		#~ print "hello2"
		#~ rospy.wait_for_service('/body_stiffness/disable')
		#~ try:
			#~ print "hello51"
			#~ service_handle2 = rospy.ServiceProxy('/body_stiffness/disable', Empty)
			#~ resp2 = service_handle2()
			#~ print "Body stiffness is disable"
		#~ except rospy.ServiceException, e:
			#~ print "Body stiffness service call failed: %s"%e
	
		
if __name__ == "__main__":
	nao= move_nao_head()
	nao.call_body_stifness_server_enable()
	nao.move(Polygon)	
	#~ nao.call_body_stifness_server_disable()	

		
