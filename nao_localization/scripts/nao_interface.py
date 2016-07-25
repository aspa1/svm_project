#!/usr/bin/env python
from rapp_robot_api import RappRobot
from sensor_msgs.msg import LaserScan
from RappCloud import QrDetection
from nao_localization.srv import SetObject
from nao_localization.srv import SetObjectResponse
from nao_localization.srv import GetObjects
from nao_localization.srv import GetObjectsResponse
from nao_localization.msg import ObjectMsg

import rospy
import sys

class NaoInterface:
	def __init__(self):
		self.rh = RappRobot()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		rospy.Timer(rospy.Duration(5), self.qrDetectionCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)
		self.s = rospy.Service('set_object', SetObject, self.setNewObjectCallback)
		self.s1 = rospy.Service('get_objects', GetObjects, self.getObjectsCallback)
		
		self.objects = {}
		self.static_objects = []
		
	def sonarsCallback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()['sonars']
		laser_msg = LaserScan()
		laser_msg.ranges.append(sonars['front_right'])
		laser_msg.ranges.append(sonars['front_left'])
		laser_msg.range_max = 1.00
		laser_msg.angle_increment = 0.785398185253
		laser_msg.angle_min = -0.392699092627
		self.pub.publish(laser_msg)
		
	def qrDetectionCallback(self, event):
		print "QrDetection"
		self.rh.vision.capturePhoto("/home/nao/test.jpg", "front", "1280x960")
		self.rh.utilities.moveFileToPC("/home/nao/test.jpg", "/home/aspa/test.jpg")
		svc = QrDetection(image="/home/aspa/test.jpg")
		response = svc.call()
		print response.serialize()
		head_yaw = self.rh.humanoid_motion.getJointAngles(["HeadYaw"])['angles'][0]
		print head_yaw
		
	def setNewObjectCallback(self, req):
		print "setNewObjectCallback"
		print req
		self.objects[req.object.message] = req.object
		res = SetObjectResponse()
		res.success = True
		return res
		
	def getObjectsCallback(self, req):
		res = GetObjectsResponse()
		if req.localization_type == "dynamic":
			for i in range(0, len(self.objects)):
				obj = ObjectMsg()
				obj.x = self.objects.values()[i].x
				obj.y = self.objects.values()[i].y
				obj.message = self.objects.values()[i].message
				obj.type = self.objects.values()[i].type
				if req.object_type == self.objects.values()[i].type or req.object_type == "all" or req.object_type == "":
					if obj.message in res.objects:
						continue
					res.objects.append(obj)
		elif req.localization_type == "static":
			for i in range(0, len(self.static_objects)):
				obj = ObjectMsg()
				obj.x = static_objects[i].x
				obj.y = static_objects[i].y
				obj.message = static_objects[i].message
				obj.type = static_objects[i].type
				res.objects.append(obj)
		elif req.localization_type == "all":
			for i in range(0, len(self.objects)):
				obj = ObjectMsg()
				obj.x = self.objects.values()[i].x
				obj.y = self.objects.values()[i].y
				obj.message = self.objects.values()[i].message
				obj.type = self.objects.values()[i].type
				if req.object_type == self.objects.values()[i].type or req.object_type == "all" or req.object_type == "":
					if obj.message in res.objects:
						continue
					res.objects.append(obj)
			for i in range(0, len(self.static_objects)):
				obj = ObjectMsg()
				obj.x = static_objects[i].x
				obj.y = static_objects[i].y
				obj.message = static_objects[i].message
				obj.type = static_objects[i].type
				res.objects.append(obj)
		else:
			print "No such type"
		res.success = True	
		print res.objects
		return res		
		
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
