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
		self.pub2 = rospy.Publisher('visualization_marker', Marker, queue_size=1)
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
		self.response = {}
		self.response_new = {}
		self.response['qr_messages'] = []
		self.response_new['qr_messages'] = []
		self.response['qr_centers'] = []
		self.response_new['qr_centers'] = []

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
		counter1 = 0
		obj_found = False
		flag = False
		
		if (self.tracking_flag == False):
			print "QrDetection"
			image = self.imageLoad()
			for symbol in image:
				self.response['qr_messages'].append(symbol.data)
				#~ print 'response[qr_messages]', self.response['qr_messages'] 
			if len(self.response['qr_messages']) <> 0:
				for qrm in self.response['qr_messages']:
					if "Localization" in qrm:
						print "Loc QR detected"
						print self.response
						print qrm
						qr_msg = RfidSensorMeasurementMsg()
						qr_msg.rfid_tags_ids.append(qrm)
						self.pub1.publish(qr_msg)
					else:
						#~ print "Object QR detected 1st time"
						#~ print qrm
						rospy.wait_for_service('robot_state')
						try:
							robot_state = rospy.ServiceProxy('robot_state', RobotState)
							resp1 = robot_state(False)
							#~ print "Movement stopped"
						except rospy.ServiceException, e:
							print "Service call failed: %s"%e
						self.tracking_flag = True
				while (len(self.response_new['qr_messages']) == 0) and counter1 < 4:
					image2 = self.imageLoad()
					for symbol in image2:
						if "Localization" not in symbol.data and symbol.data not in self.response_new['qr_messages']:
							self.counter += 1
							flag = True
							self.response_new['qr_messages'].append(symbol.data)
							x = (symbol.location[3][0] + symbol.location[2][0])/2 
							y = (symbol.location[0][1] + symbol.location[3][1])/2
							self.response_new['qr_centers'].append((x, y)) 
						print "size: ", len(self.response_new['qr_messages'])
						counter1 += 1
				#~ if len(self.response_new['qr_messages']) <> 0:
				if flag == True:
					for i in range (len(self.response_new['qr_messages']) - self.counter, len(self.response_new['qr_messages'])):
						print 'i = ', i
						rospy.wait_for_service('set_behavior')
						try:
							print "Object found 2nd time"
							edge = 180
							set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
							polygon = Polygon()
							qr_center = Point32()
							qr_center2 = Point32()
							qr_center.x = (self.response_new['qr_centers'][i][0] - edge/2)/2
							qr_center.y = (self.response_new['qr_centers'][i][1] - edge/2)/2
							while (qr_center.x + (edge/2)) > (640 / 2) or (qr_center.y + (edge/2))> (480 / 2):
								edge -= 80
								qr_center.x = (self.response_new['qr_centers'][i][0] - edge/2)/2
								qr_center.y = (self.response_new['qr_centers'][i][1] - edge/2)/2
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
							print "TRACKING object ", self.response_new['qr_messages'][i]
							while self.relative_obj_x == 0.0 and self.relative_obj_y == 0.0:
								self.tracking_flag = True
							print self.relative_obj_x
							print "Finished tracking ", self.response_new['qr_messages'][i]
							self.setObjectClient(i)
							self.visualize()
							
						except rospy.ServiceException, e:
							print "Service call failed: %s"%e
					self.tracking_flag = False
						
				else:
					#~ print "Object lost"
					self.tracking_flag = False


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
		obj.message = self.response_new['qr_messages'][i]
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
		if req.localization_type == "Dynamic":
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
		elif req.localization_type == "Static":
			for i in range(0, len(self.static_objects)):
				obj = ObjectMsg()
				obj.x = static_objects[i].x
				obj.y = static_objects[i].y
				obj.message = static_objects[i].message
				obj.type = static_objects[i].type
				res.objects.append(obj)
		elif req.localization_type == "All":
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
		for i in range(0, len(self.objects)):
		#~ counter = 0
			print "Visualization"
			print len(self.response_new['qr_messages'])
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
