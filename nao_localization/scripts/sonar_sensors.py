#!/usr/bin/env python
from rapp_robot_api import RappRobot

import rospy
import sys

class SonarsSense:
	def __init__(self):
		self.rh = RappRobot()
		rospy.Timer(rospy.Duration(0.1), self.sonarsCallback)
		self.pub = rospy.Publisher('inner/sonar_measurements', LaserScan, queue_size=1)

	def sonarsCallback(self, event):
		sonars = self.rh.sensors.getSonarsMeasurements()
		
if __name__ == "__main__":
	rospy.init_node('nao_rosmsg', anonymous=True)
	nao = SonarsSense()
	rospy.spin()
