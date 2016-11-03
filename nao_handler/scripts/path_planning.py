#!/usr/bin/env python

from ogmpp_communications.srv import OgmppPathPlanningSrv
from ogmpp_communications.srv import OgmppPathPlanningSrvRequest
from ogmpp_communications.srv import OgmppPathPlanningSrvResponse

import rospy
import sys

class PathPlanning:
	def __init__(self):
		self.path_publisher = rospy.Publisher(rospy.get_param('path_pub_topic'), \
			Path, queue_size = 10)
		self.service_path = rospy.Service('get_path', GetPath, self.get_path)

	def get_path(self, request):
		print "Waiting for path service"
		rospy.wait_for_service('/ogmpp_path_planners/plan')
		print "Service ok"
		try:
			pathSrv = rospy.ServiceProxy('/ogmpp_path_planners/plan', OgmppPathPlanningSrv)
			path = OgmppPathPlanningSrvRequest()
			path.method = "uniform_prm"
			path.data.begin.x = self.robot_x
			path.data.begin.y = self.robot_y
			path.data.end.x = request.x
			path.data.end.y = request.y
			
			pathRes = pathSrv(path)
			#~ print pathRes
			
			ros_path = Path()
			
			ros_path.header.frame_id = "map"
			_map = OccupancyGrid()
			
			for p in self.path:
				ps = PoseStamped()
				ps.header.frame_id = "map"
				ps.pose.position.x = p[0] * _map.data.info.resolution + \
					_map.data.info.origin.position.x
				ps.pose.position.y = p[1] * _map.data.info.resolution + \
					_map.data.info.origin.position.y
				print ps
				ros_path.poses.append(ps)
			
			#~ self.path_publisher.publish(ros_path)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		res = GetPath()
		res.success = True
		return True
		
if __name__ == "__main__":
	rospy.init_node('path_planning', anonymous=True)
	nao = PathPlanning()
	rospy.spin()	
