#!/usr/bin/env python
from nao_localization.srv import SetObject
from nao_localization.msg import ObjectMsg

import sys
import rospy

def	setObjectClient():
	obj = ObjectMsg()
	obj.x = x
	obj.y = y
	obj.message = msg
	obj.type = type
	rospy.wait_for_service('set_object')
	try:
		set_object = rospy.ServiceProxy('set_object', SetObject)
		resp1 = set_object(obj)
		return resp1.success
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

if __name__ == "__main__":
	if len(sys.argv) == 5:
		x = int(sys.argv[1])
		y = int(sys.argv[2])
		msg = str(sys.argv[3])
		type = str(sys.argv[4])
	print setObjectClient()
