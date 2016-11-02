#!/usr/bin/env python

import sys
import rospy
from nao_handler.srv import *
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

def set_behavior_test():
    rospy.wait_for_service('set_behavior')
    try:
        set_behavior = rospy.ServiceProxy('set_behavior', SetBehavior)
        polygon = Polygon()
        print "aoua"
        #~ behavior = "track_bounding_box"
        behavior = "obstacle_avoidance"
        print polygon
        point = Point32()
        point.x = 100
        point.y = 100
        polygon.points.append(point)
        polygon.points.append(point)
        
        #~ behavior = "obstacle_avoidance"
        resp1 = set_behavior(behavior, polygon)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#~ def usage():
    #~ return "%s [behavior polygon]"%sys.argv[0]

if __name__ == "__main__":
    #~ if len(sys.argv) == 3:
        #~ behavior = str(sys.argv[1])
        #~ 
    #~ else:
        #~ print usage()
        #~ sys.exit(1)
    #~ print "Requesting behavior + polygon"
    print set_behavior_test()
