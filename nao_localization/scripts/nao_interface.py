#!/usr/bin/env python
from rapp_robot_api import RappRobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from sensor_msgs.msg import LaserScan
from stdr_msgs.msg import RfidSensorMeasurementMsg
from RappCloud import RappPlatformAPI
from nao_localization.srv import SetObject
from nao_localization.srv import SetObjectResponse
from nao_localization.srv import GetObjects
from nao_localization.srv import GetObjectsResponse
from nao_handler.srv import *
from nao_localization.msg import ObjectMsg

import rospkg
import rospy
import sys
import tf
import zbar
import zbarlight
import math
import Image

class NaoInterface:
	def __init__(self):
		self.rh = RappRobot()
		self.ch = RappPlatformAPI()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		rospy.Timer(rospy.Duration(0.1), self.getRobotPositionCallback)
		rospy.Timer(rospy.Duration(0.5), self.qrDetectionCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)
		self.pub1 = rospy.Publisher('/inner/qr_detection', RfidSensorMeasurementMsg, queue_size=1)
		self.sub = rospy.Subscriber("/relative_object_position", Twist, self.getObjectPositioncallback)
		self.s = rospy.Service('set_object', SetObject, self.setNewObjectCallback)
		self.s1 = rospy.Service('get_objects', GetObjects, self.getObjectsCallback)
		self.listener = tf.TransformListener()
		self.robot_x = 0
		self.robot_y = 0
		self.robot_th = 0
		self.relative_obj_x = 0
		self.relative_obj_y = 0
		self.tracking_flag = False
		self.objects = {}
		self.static_objects = []
		self.counter = 0

		
	def sonarsCallback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()['sonars']
		laser_msg = LaserScan()
		laser_msg.ranges.append(sonars['front_right'])
		laser_msg.ranges.append(sonars['front_left'])
		laser_msg.range_max = 1.00
		laser_msg.angle_increment = 0.785398185253
		laser_msg.angle_min = -0.392699092627
		self.pub.publish(laser_msg)
		
	def imageLoad(self):
		rospack = rospkg.RosPack()
		img_path = rospack.get_path('nao_localization') + "/cfg/nao_capture.jpg"
		self.rh.vision.capturePhoto("/home/nao/test.jpg", "front", "640x480")
		self.rh.utilities.moveFileToPC("/home/nao/test.jpg", img_path)
		image = Image.open(img_path)
		image.load()
		converted_image = image.convert('L')
		
		raw = converted_image.tobytes()
		width, height = converted_image.size
		
		image2 = zbar.Image(width, height, 'Y800', raw)
		scanner = zbar.ImageScanner()
		scanner.scan(image2)
		return image2
		
		
	def qrDetectionCallback(self, event):
		self.counter = 0
		obj_found = False
		
		response = {}
		response_new = {}
		response['qr_messages'] = []
		response_new['qr_messages'] = []
		response['qr_centers'] = []
		response_new['qr_centers'] = []

		if (self.tracking_flag == False):
			print "QrDetection"
			image = self.imageLoad()
			for symbol in image:
				response['qr_messages'].append(symbol.data)
				print 'response[qr_messages]', response['qr_messages'] 
			if len(response['qr_messages']) <> 0:
				for qrm in response['qr_messages']:
					if "Localization" in qrm:
						print "Loc QR detected"
						print response
						print qrm
						qr_msg = RfidSensorMeasurementMsg()
						qr_msg.rfid_tags_ids.append(qrm)
						self.pub1.publish(qr_msg)
					else:
						print "Object QR detected 1st time"
						print qrm
						rospy.wait_for_service('robot_state')
						try:
							robot_state = rospy.ServiceProxy('robot_state', RobotState)
							resp1 = robot_state(False)
							print "Movement stopped"
						except rospy.ServiceException, e:
							print "Service call failed: %s"%e
						self.tracking_flag = True
				while (len(response_new['qr_messages']) == 0) and self.counter < 4:
					image2 = self.imageLoad()
					for symbol in image2:
						response_new['qr_messages'].append(symbol.data)
						x = (symbol.location[0][0] + symbol.location[1][0])/2 
						y = (symbol.location[0][1] + symbol.location[3][1])/2
						response_new['qr_centers'].append((x, y)) 
					print response_new
					self.counter += 1
				if len(response_new['qr_messages']) <> 0:
					rospy.wait_for_service('set_behavior')
					try:
						print "Object found 2nd time"
						edge = 150
						set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
						polygon = Polygon()
						qr_center = Point32()
						qr_center2 = Point32()
						qr_center.x = (response_new['qr_centers'][0][0] - edge/2)/2
						qr_center.y = (response_new['qr_centers'][0][1] - edge/2)/2
						while (qr_center.x + (edge/2)) > (640 / 2) or (qr_center.y + (edge/2))> (480 / 2):
							edge -= 60
							qr_center.x = (response_new['qr_centers'][0][0] - edge/2)/2
							qr_center.y = (response_new['qr_centers'][0][1] - edge/2)/2
						polygon.points.append(qr_center)
						qr_center2.x = (edge)/2
						qr_center2.y = (edge)/2
						polygon.points.append(qr_center2)
						print polygon
						try:
							robot_state = rospy.ServiceProxy('robot_state', RobotState)
							resp1 = robot_state(True)
						except rospy.ServiceException, e:
							print "Service call failed: %s"%e
						behavior = "track_bounding_box"
						resp2 = set_behavior(behavior, polygon)
						print "TRACKING"
						while self.relative_obj_x == 0.0 and self.relative_obj_y == 0.0:
							self.tracking_flag = True
						print self.relative_obj_x
						print "Finished tracking"
						self.tracking_flag = False
					except rospy.ServiceException, e:
						print "Service call failed: %s"%e
				else:
					print "Object lost"
					self.tracking_flag = False


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
		pass
		#~ try:
			#~ (trans,rot) = self.listener.lookupTransform('/map', '/nao_pose', rospy.Time(0))
			#~ self.robot_x = trans[0]
			#~ self.robot_y = trans[1]
			#~ self.robot_th = rot[0]
		#~ except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
			#~ print ex
		
	def getObjectPositioncallback(self, object_pos):
		self.relative_obj_x = object_pos.linear.x
		self.relative_obj_y = object_pos.linear.y
		self.absolute_obj_x = math.cos(-self.robot_th) * self.relative_obj_x - \
			math.sin(-self.robot_th) * self.relative_obj_y + self.robot_x
		self.absolute_obj_y = math.sin(-self.robot_th) * self.relative_obj_x + \
			math.cos(-self.robot_th) * self.relative_obj_y + self.robot_y
	
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
