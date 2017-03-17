#!/usr/bin/env python
from rapp_robot_api import RappRobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon
from sensor_msgs.msg import LaserScan
from stdr_msgs.msg import RfidSensorMeasurementMsg
from visualization_msgs.msg import Marker
from RappCloud import RappPlatformAPI
from nao_localization.srv import SetObject
from nao_localization.srv import SetObjectResponse
from nao_localization.srv import GetObjects
from nao_localization.srv import GetObjectsResponse
from nao_handler.srv import *
from nao_localization.msg import ObjectMsg
from nao_handler.msg import TrackingFlag

import rospkg
import rospy
import sys
import tf
import zbar
import zbarlight
import math
import Image
import time

class NaoInterface:
	def __init__(self):
		self.rh = RappRobot()
		self.ch = RappPlatformAPI()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		rospy.Timer(rospy.Duration(0.1), self.getRobotPositionCallback)
		rospy.Timer(rospy.Duration(2), self.qrDetectionCallback)
		rospy.Timer(rospy.Duration(15), self.stopRobotCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)
		self.pub1 = rospy.Publisher('/inner/qr_detection', RfidSensorMeasurementMsg, queue_size=1)
		self.pub2 = rospy.Publisher('visualization_marker', Marker, queue_size=1)
		self.sub = rospy.Subscriber("/relative_object_position", Twist, self.objectHandlerCallback)
		self.sub1 = rospy.Subscriber('/tracking_flag', TrackingFlag, self.trackingStateCallback)
		self.s = rospy.Service('set_object', SetObject, self.setNewObjectCallback)
		self.s1 = rospy.Service('get_objects', GetObjects, self.getObjectsCallback)
		self.listener = tf.TransformListener()
		self.robot_x = 0
		self.robot_y = 0
		self.robot_th = 0

		self.objects = {}
		
		self.stop = False
		#~ self.beginObstacleAvoidance()

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
		#~ print "imageLoad"
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
		#~ print "Image loaded"
		return image2
		
		
	def qrDetectionCallback(self, event):
	
		if (self.stop == False):
			flag = False
			temp_loc = []
			temp_obj = []
			print "QrDetection"
			
			
			image = self.imageLoad()
			for symbol in image:
				if "Localization" in symbol.data: 
					temp_loc.append(symbol.data)
				else:
					self.stop = True
					rospy.wait_for_service('robot_state')
					try:
						robot_state = rospy.ServiceProxy('robot_state', RobotState)
						resp1 = robot_state(False)
						print "Movement stopped"
					except rospy.ServiceException, e:
						print "Service call failed: %s"%e
					temp_obj.append(symbol.data)
				
			for i in range (0, len(temp_loc)):
				print "Loc QR detected"
				qr_msg = RfidSensorMeasurementMsg()
				qr_msg.rfid_tags_ids.append(temp_loc[i])
				self.pub1.publish(qr_msg)
				
			if len(temp_obj) <> 0:
				for i in range (0, len(temp_obj)):
					print "temp_obj 1st time", temp_obj[i]

				image2 = self.imageLoad()
				for symbol in image2:
					if flag == True:
						break
					for i in range (0, len(temp_obj)):

						if symbol.data in temp_obj[i] and temp_obj[i] not in self.objects:
							print "symbol.data", symbol.data
							self.obj = temp_obj[i]
							self.x = (symbol.location[3][0] + symbol.location[1][0])/2 
							self.y = (symbol.location[0][1] + symbol.location[2][1])/2
							flag = True
							print "flag =", flag

							break
				#~ print self.response
				if flag == True:
					#~ print "blah ", i
					rospy.wait_for_service('set_behavior')
					try:
						print "Object ", self.obj, " found 2nd time"
						edge = 250
						set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
						polygon = Polygon()
						qr_center = Point32()
						qr_center2 = Point32()
						qr_center.x = (self.x - edge/2)/2
						qr_center.y = (self.y - edge/2)/2

						while (qr_center.x + (edge/2)) > (640 / 2) or (qr_center.y + (edge/2))> (480 / 2) or (qr_center.x + (edge/2)) < 0 or (qr_center.y + (edge/2))< 0:
							edge -= 40
							qr_center.x = (self.x - edge/2)/2
							qr_center.y = (self.y - edge/2)/2
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
						print "TRACKING object ", self.obj
						self.tracking_flag = True
						while self.tracking_flag == True:
							time.sleep(0.01)
							self.stop = True
						print "Stopped tracking"
						#~ self.beginObstacleAvoidance()
						rospy.wait_for_service('set_behavior')
						try:
							try:
								robot_state = rospy.ServiceProxy('robot_state', RobotState)
								resp1 = robot_state(False)
								#~ print "OBSTACLE"
							except rospy.ServiceException, e:
								print "Service call failed: %s"%e
						except rospy.ServiceException, e:
							print "Service call failed: %s"%e
					except rospy.ServiceException, e:
						print "Service call failed: %s"%e
			self.stop = False
			
	
	def beginObstacleAvoidance(self):
		self.stop = False
		rospy.wait_for_service('set_behavior')
		try:
			try:
				robot_state = rospy.ServiceProxy('robot_state', RobotState)
				resp1 = robot_state(True)
				print "OBSTACLE"
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
				
			set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
			polygon = Polygon()
			qr_center = Point32()
			qr_center2 = Point32()
			qr_center.x = 100
			qr_center.y = 100
			polygon.points.append(qr_center)
			qr_center2.x = 100
			qr_center2.y = 100
			polygon.points.append(qr_center2)
			#~ print polygon
			behavior = "obstacle_avoidance"
			resp2 = set_behavior(behavior, polygon)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		
		
	def stopRobotCallback(self,event):
		if self.stop == True:
			return
		try:
			robot_state = rospy.ServiceProxy('robot_state', RobotState)
			resp1 = robot_state(False)
			print "Wait"
		except rospy.ServiceException, e:
			print "Service call failed: %s"
		time.sleep(5)
		try:
			robot_state = rospy.ServiceProxy('robot_state', RobotState)
			resp1 = robot_state(True)
			print "Go Go Go"
		except rospy.ServiceException, e:
			print "Service call failed: %s"
			
	def trackingStateCallback(self, flag):
		self.tracking_flag = flag.tracking_flag 
		
	def setNewObjectCallback(self, req):
		print "setNewObjectCallback"
		print req
		self.objects[req.object.message] = req.object
		res = SetObjectResponse()
		res.success = True
		return res

	def setObjectClient(self):
		obj = ObjectMsg()
		obj.x = self.absolute_obj_x
		obj.y = self.absolute_obj_y
		obj.message = self.obj
		obj.type = 'Dynamic'
		rospy.wait_for_service('set_object')
		try:
			set_object = rospy.ServiceProxy('set_object', SetObject)
			resp1 = set_object(obj)
			return resp1.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def getObjectsCallback(self, req):
		res = GetObjectsResponse()
		if req.type == "all" or req.type == "":
			for i in range(0, len(self.objects)):
				obj = ObjectMsg()
				obj.x = self.objects.values()[i].x
				obj.y = self.objects.values()[i].y
				obj.message = self.objects.values()[i].message
				obj.type = self.objects.values()[i].type
				if obj.message in res.objects:
					continue
				res.objects.append(obj)
		else:
			for i in range(0, len(self.objects)):
				if req.type in self.objects.values()[i].message:
					obj = ObjectMsg()
					obj.x = self.objects.values()[i].x
					obj.y = self.objects.values()[i].y
					obj.message = self.objects.values()[i].message
					obj.type = self.objects.values()[i].type
					if obj.message in res.objects:
						continue
					res.objects.append(obj)
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
			pass
			#~ print ex
		
	def objectHandlerCallback(self, object_pos):
		relative_obj_x = object_pos.linear.x
		relative_obj_y = object_pos.linear.y
		self.absolute_obj_x = math.cos(-self.robot_th) * relative_obj_x - \
			math.sin(-self.robot_th) * relative_obj_y + self.robot_x
		self.absolute_obj_y = math.sin(-self.robot_th) * relative_obj_x + \
			math.cos(-self.robot_th) * relative_obj_y + self.robot_y
			
		print "Successfully tracked ", self.obj

		self.rh.audio.speak("Object")
		self.rh.audio.speak(self.obj)
		self.setObjectClient()
		self.visualize()
	
	def visualize(self):
		print "Visualization"
		print len(self.objects)
		for i in range(0, len(self.objects)):
			m = Marker()
			m.header.frame_id = "map";
			m.type = m.SPHERE_LIST
			m.action = m.ADD
			m.id = i
			m.ns = "Object_QRs"
			m.scale.x = 0.4
			m.scale.y = 0.4
			m.scale.z = 0.4
			m.color.a = 1.0
			m.color.r = 1.0
			m.color.g = 0.0
			m.color.b = 1.0
						
			p = Point32()
			p.x = self.objects.values()[i].x
			p.y = self.objects.values()[i].y
			print "Point: ", p.x, p.y 
			m.points.append(p)
			self.pub2.publish(m)
			print p
			print "Object Marker Published"
		
			m1 = Marker()
			m1.header.frame_id = "map"
			m1.type = m1.TEXT_VIEW_FACING
			m1.action = m1.ADD
			m1.id = i
			m1.ns = "Object_QR_id"
			m.scale.x = 0.65
			m.scale.y = 0.65
			m1.scale.z = 0.65
			m1.color.a = 1.0
			m1.color.r = 0.0
			m1.color.g = 0.0
			m1.color.b = 0.0
			m1.text = self.objects.values()[i].message
			m1.pose.position.x = self.objects.values()[i].x
			m1.pose.position.y = self.objects.values()[i].y
			p1 = Point32()
			p1.x = self.objects.values()[i].x
			p1.y = self.objects.values()[i].y
			m1.points.append(p1)
			
			self.pub2.publish(m1)
		
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
