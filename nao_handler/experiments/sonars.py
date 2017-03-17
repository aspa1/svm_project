#!/usr/bin/env python
import time
import rospy

from rapp_robot_api import RappRobot

class sonars():
	def __init__(self):
		self.rh = RappRobot()
		self.sonars_array = []
		self.sonars_timer = rospy.Timer(rospy.Duration(0.1), self.callback)
	

	def callback(self,event):
		
		sonars = self.rh.sensors.getSonarsMeasurements()['sonars']
			
		self.sonars_array.append(sonars)
		print "sonars_array", self.sonars_array
		with open('/home/chrisa/catkin_ws/src/thesis/nao_handler/sonars_exp8.txt', 'w') as output:
			for i in range(0,len(self.sonars_array)):
				output.write('%f %f \n' % (self.sonars_array[i]['front_left'], self.sonars_array[i]['front_right']))
		
		print "sonars['front_left']:", sonars['front_left']
		print "sonars['front_right']:", sonars['front_right']

if __name__ == "__main__":
	rospy.init_node('nao_motion', anonymous=True)
	s = sonars()
	rospy.spin()	

