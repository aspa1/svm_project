#!/usr/bin/env python
from rapp_robot_api import RappRobot
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from stdr_msgs.msg import RfidSensorMeasurementMsg
from RappCloud import RappPlatformAPI
from nao_localization.srv import SetObject
from nao_localization.srv import SetObjectResponse
from nao_localization.srv import GetObjects
from nao_localization.srv import GetObjectsResponse
from nao_localization.msg import ObjectMsg

import rospkg
import rospy
import sys
import tf

class NaoInterface:
	def __init__(self):
		self.rh = RappRobot()
		self.ch = RappPlatformAPI()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		rospy.Timer(rospy.Duration(0.1), self.getRobotPositionCallback)
		rospy.Timer(rospy.Duration(5), self.qrDetectionCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)
		self.pub1 = rospy.Publisher('/inner/qr_detection', RfidSensorMeasurementMsg, queue_size=1)
		self.sub = rospy.Subscriber("/relative_object_position", Twist, self.getObjectPositioncallback)
		self.s = rospy.Service('set_object', SetObject, self.setNewObjectCallback)
		self.s1 = rospy.Service('get_objects', GetObjects, self.getObjectsCallback)
		self.listener = tf.TransformListener()
		self.robot_x = 0
		self.robot_y = 0
		self.robot_th = 0
		
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
		rospack = rospkg.RosPack()
		img_path = rospack.get_path('nao_localization') + "/cfg/nao_capture.jpg"
		self.rh.vision.capturePhoto("/home/nao/test.jpg", "front", "640x480")
		print img_path
		self.rh.utilities.moveFileToPC("/home/nao/test.jpg", img_path)
		#svc = QrDetection(imageFilepath="/home/chrisa/test.jpg")
		response = self.ch.qrDetection(img_path)
		print response
		print response['qr_messages']
		if "Localization" in response['qr_messages']:
			qr_msg = RfidSensorMeasurementMsg()
			qr_msg.rfid_tags_ids.append(response['qr_messages'])
			self.pub1.publish(qr_msg)
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
		
	def getRobotPositionCallback(self, event):
		#~ pass
		try:
			(trans,rot) = self.listener.lookupTransform('/map', '/nao_pose', rospy.Time(0))
			self.robot_x = trans[0]
			self.robot_y = trans[1]
			self.robot_th = rot[0]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
			print ex
		
	def getObjectPositioncallback(self, event):
		relative_obj_x = twist.linear.x
		relative_obj_y = twist.linear.y
		self.absolute_obj_x = cos(-self.robot_th) * relative_obj_x - \
			sin(-self.robot_th) * relative_obj_y + self.robot_x
		self.absolute_obj_y = sin(-self.robot_th) * relative_obj_x + \
			cos(-self.robot_th) * relative_obj_y + self.robot_y
	
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
