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
from nao_handler.msg import LostObject

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
		rospy.Timer(rospy.Duration(3), self.qrDetectionCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)
		self.pub1 = rospy.Publisher('/inner/qr_detection', RfidSensorMeasurementMsg, queue_size=1)
		self.pub2 = rospy.Publisher('visualization_marker', Marker, queue_size=1)
		self.sub = rospy.Subscriber("/relative_object_position", Twist, self.getObjectPositioncallback)
		self.sub1 = rospy.Subscriber('/lost_object', LostObject, self.lostObjectCallback)
		self.s = rospy.Service('set_object', SetObject, self.setNewObjectCallback)
		self.s1 = rospy.Service('get_objects', GetObjects, self.getObjectsCallback)
		self.listener = tf.TransformListener()
		self.robot_x = 0
		self.robot_y = 0
		self.robot_th = 0
		self.relative_obj_x = 0
		self.relative_obj_y = 0
		self.stop = False
		self.lost_obj = False
		self.objects = {}
		#~ self.loc_qrs = {}
		#~ self.loc_qrs['qr_messages'] = []
		self.static_objects = []
		self.counter = 0
		self.response = {}
		#~ self.response_new = {}
		self.response['qr_messages'] = []
		#~ self.response_new['qr_messages'] = []
		self.response['qr_centers'] = []
		#~ self.response_new['qr_centers'] = []
		self.temp_obj = []

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
		print "imageLoad"
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
		print "Image loaded"
		return image2
		
		
	def qrDetectionCallback(self, event):
		counter1 = 0
		obj_found = False
		flag = False
		new_obj = False
		temp_loc = []
		
		
		if (self.stop == False):
			now = rospy.get_rostime()
			print "QrDetection"
			print "Response", self.response

			#~ rospy.wait_for_service('set_behavior')
			#~ try:
				#~ try:
					#~ robot_state = rospy.ServiceProxy('robot_state', RobotState)
					#~ resp1 = robot_state(True)
					#~ print "OBSTACLE"
				#~ except rospy.ServiceException, e:
					#~ print "Service call failed: %s"%e
					
				#~ self.rh.humanoid_motion.setJointAngles(["HeadPitch"],[-0.05], 0.1)
				#~ set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
				#~ polygon = Polygon()
				#~ qr_center = Point32()
				#~ qr_center2 = Point32()
				#~ qr_center.x = 100
				#~ qr_center.y = 100
				#~ polygon.points.append(qr_center)
				#~ qr_center2.x = 100
				#~ qr_center2.y = 100
				#~ polygon.points.append(qr_center2)
				#~ print polygon
				#~ behavior = "obstacle_avoidance"
				#~ resp2 = set_behavior(behavior, polygon)
				#~ return resp1.sum
			#~ except rospy.ServiceException, e:
				#~ print "Service call failed: %s"%e
			
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
					self.temp_obj.append(symbol.data)
				
					
			for i in range (0, len(temp_loc)):
				print "Loc QR detected"
				qr_msg = RfidSensorMeasurementMsg()
				qr_msg.rfid_tags_ids.append(temp_loc[i])
			if len(temp_loc) <> 0:
				self.pub1.publish(qr_msg)
			if len(self.temp_obj) <> 0:
				self.stop = True
				
				
				#~ for i in range (0, len(self.temp_obj)):
					#~ print self.temp_obj[i]
				image2 = self.imageLoad()
				for symbol in image2:
					for i in range (0, len(self.temp_obj)):
						print "temp_obj ", self.temp_obj[i]
						if symbol.data in self.temp_obj[i] and self.temp_obj[i] not in self.response['qr_messages']:
							print symbol.data
							x = (symbol.location[3][0] + symbol.location[1][0])/2 
							y = (symbol.location[0][1] + symbol.location[2][1])/2
							#~ self.response['qr_messages'].append(symbol.data)
							#~ self.response['qr_centers'].append((x,y))
							flag = True
							print "flag=", flag
						if flag == True:
							print "i = ", i
							#~ i -= 1
							#~ print temp_obj[i]
							break
				print self.response
				print "temp_obj2 ", self.temp_obj[i]
				#~ for i in range (0, len(self.response['qr_messages'])):
				#~ if len(self.response['qr_messages']) <> 0:
				if flag == True:
					print "blah ", i
					print self.temp_obj[i]
					print x
					print y
					self.stop = True
					#~ i = len(self.response['qr_messages']) - 1
					#~ print "Length = ", i + 1
					#~ print self.response['qr_messages'][i]
					rospy.wait_for_service('set_behavior')
					try:
						print "Object ", self.temp_obj[i], " found 2nd time"
						#~ print "Object ", self.response['qr_messages'][i], " found 2nd time"
						#~ self.lost_object_counter = 8
						edge = 160
						set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
						polygon = Polygon()
						qr_center = Point32()
						qr_center2 = Point32()
						qr_center.x = (x - edge/2)/2
						qr_center.y = (y - edge/2)/2
						#~ qr_center.x = (self.response['qr_centers'][i][0] - edge/2)/2
						#~ qr_center.y = (self.response['qr_centers'][i][1] - edge/2)/2
						while (qr_center.x + (edge/2)) > (640 / 2) or (qr_center.y + (edge/2))> (480 / 2):
							edge -= 20
							qr_center.x = (x - edge/2)/2
							qr_center.y = (y - edge/2)/2
							#~ qr_center.x = (self.response['qr_centers'][i][0] - edge/2)/2
							#~ qr_center.y = (self.response['qr_centers'][i][1] - edge/2)/2
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
						#~ self.counter = 15
						#~ print "TRACKING object ", self.response['qr_messages'][i]
						print "TRACKING object ", self.temp_obj[i]
						
						while self.relative_obj_x == 0.0 and self.relative_obj_y == 0.0:
							if self.lost_obj == False:
								self.stop = True
								#~ print "Tracking in progress"
							else:
								print "Object lost"
								self.stop = False
								break
						if self.stop == True:
							print self.relative_obj_x
							#~ print "Successfully tracked ", self.response['qr_messages'][i]
							print "Successfully tracked ", self.temp_obj[i]
							self.response['qr_messages'].append(self.temp_obj[i])
							self.response['qr_centers'].append((x,y))
							self.rh.audio.speak("Object")
							self.rh.audio.speak(self.temp_obj[i])
							self.setObjectClient(i)
							self.visualize()
						
					except rospy.ServiceException, e:
						print "Service call failed: %s"%e
			self.stop = False

	def lostObjectCallback(self, flag):
		self.lost_obj = flag.lost_object 
		
	def setNewObjectCallback(self, req):
		print "setNewObjectCallback"
		print req
		self.objects[req.object.message] = req.object
		res = SetObjectResponse()
		res.success = True
		return res

	def setObjectClient(self, i):
		obj = ObjectMsg()
		obj.x = self.absolute_obj_x
		obj.y = self.absolute_obj_y
		#~ obj.message = self.response['qr_messages'][i]
		obj.message = self.temp_obj[i]
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
		if req.type == "dynamic":
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
		elif req.type == "static":
			for i in range(0, len(self.static_objects)):
				obj = ObjectMsg()
				obj.x = static_objects[i].x
				obj.y = static_objects[i].y
				obj.message = static_objects[i].message
				obj.type = static_objects[i].type
				res.objects.append(obj)
		elif req.type == "all":
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
			pass
			#~ print ex
		
	def getObjectPositioncallback(self, object_pos):
		self.relative_obj_x = object_pos.linear.x
		self.relative_obj_y = object_pos.linear.y
		self.absolute_obj_x = math.cos(-self.robot_th) * self.relative_obj_x - \
			math.sin(-self.robot_th) * self.relative_obj_y + self.robot_x
		self.absolute_obj_y = math.sin(-self.robot_th) * self.relative_obj_x + \
			math.cos(-self.robot_th) * self.relative_obj_y + self.robot_y
	
	def visualize(self):
		print "Visualization"
		print len(self.objects)
		for i in range(0, len(self.objects)):
		#~ counter = 0
			m = Marker()
			m.header.frame_id = "map";
			#~ m.header.stamp = ros::Time()
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
			#~ p.x = self.response_new['qr_centers'][i][0]
			#~ p.y = self.response_new['qr_centers'][i][1]
			p.x = self.objects.values()[i].x
			p.y = self.objects.values()[i].y
			print "Point: ", p.x, p.y 
			m.points.append(p)
			self.pub2.publish(m)
			print p
			print "Object Marker Published"
		
			m1 = Marker()
			m1.header.frame_id = "map"
			#~ m1.header.stamp = ros::Time();
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
			#~ counter += 1
		
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
