#!/usr/bin/env python
from rapp_robot_api import RappRobot
from sensor_msgs.msg import LaserScan
#~ from RappCloud import QrDetection

import rospy
import sys

class NaoInterface:
	def __init__(self):
		self.rh = RappRobot()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		rospy.Timer(rospy.Duration(5), self.qrDetectionCallback)
		self.pub = rospy.Publisher('/inner/sonar_measurements', LaserScan, queue_size=1)

	def sonarsCallback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()[0]
		laser_msg = LaserScan()
		laser_msg.ranges.append(sonars['front_right'])
		laser_msg.ranges.append(sonars['front_left'])
		laser_msg.range_max = 2.55
		laser_msg.angle_increment = 0.785398185253
		laser_msg.angle_min = -0.392699092627
		self.pub.publish(laser_msg)
		
	#~ def qrDetectionCallback(self, event):
		#~ print "QrDetection"
		#~ self.rh.vision.capturePhoto("/home/nao/test.jpg", "front", "1280x960")
		#~ self.rh.utilities.moveFileToPC("/home/nao/test.jpg", "/home/aspa/test.jpg")
		#~ svc = QrDetection(image="/home/aspa/test.jpg")
		#~ response = svc.call()
		#~ print response.serialize()
		#~ head_yaw = self.rh.humanoid_motion.getJointAngles(["HeadYaw"])[0]
		#~ print head_yaw

		
if __name__ == "__main__":
	rospy.init_node('nao_interface_node', anonymous=True)
	nao = NaoInterface()
	rospy.spin()
