#!/usr/bin/env python

from rapp_robot_api import RappRobot
from geometry_msgs.msg import Polygon
#~ from home.chrisa.catkin_ws.src.thesis.nao_handler.scripts import move_head_with_predator

import rospy
import sys
import time

class Move:
	def __init__(self):
		
		
		rh.motion.enableMotors()
		#~ rospy.Subscriber("/vision/predator_alert", Polygon, self.walk)
		rh.humanoid_motion.goToPosture("Stand", 0.7)
		time.sleep(1)
	
	def walk(self):
		
		rh.motion.moveTo(0.2, 0.0, 0.0)
		
		print rh.sensors.getSonarsMeasurements()
		
		if rh.sensors.getSonarsMeasurements()[0]['front_left'] <= 0.6:
			print "hello"
			rh.humanoid_motion.goToPosture("Sit", 0.7)
			rh.motion.disableMotors()
		#~ rh.humanoid_motion.goToPosture("Sit", 0.7)
		#~ rh.motion.disableMotors()
		time.sleep(1)
				
		
if __name__ == "__main__":
	rospy.init_node('nao_walk', anonymous=True)
	rh = RappRobot()
	#~ rh.humanoid_motion.goToPosture("Stand", 0.7)
	nao = Move()
	rospy.spin()	

