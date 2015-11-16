#!/usr/bin/env python

from svm_project.srv import *
import rospy
import sys


p = '/samples/positives'
n = '/samples/negatives'

def call_svm_server(p, n):
    rospy.wait_for_service('image_receiver')
    try:
        service_handle = rospy.ServiceProxy('image_receiver', trainSvmSrv)
        resp1 = service_handle(p, n)
        print "Service called"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp1.success
        
print call_svm_server(p,n)
        
#~ 
#~ if __name__ == "__main__":
	#~ p = '/samples/positives'
	#~ n = '/samples/negatives'
	#~ print "Requesting %s"%(p)
	#~ print "Requesting %s"%(n)
	#~ print "Responding %s"%(call_svm_server(p, n))
