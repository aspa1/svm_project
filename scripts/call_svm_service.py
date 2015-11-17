#!/usr/bin/env python

from svm_project.srv import *
import rospy
import sys


p = '/samples/positives'
n = '/samples/negatives'

def call_image_receiver_server(p, n):
    rospy.wait_for_service('image_receiver')
    try:
        service_handle = rospy.ServiceProxy('image_receiver', trainSvmSrv)
        resp1 = service_handle(p, n)
        print "Service called"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp1.success
        
print call_image_receiver_server(p,n)

def call_url_retriever_server(x):
    rospy.wait_for_service('url_retriever')
    try:
        service_handle2 = rospy.ServiceProxy('url_retriever', urlRetrieverSrv)
        resp2 = service_handle2(x)
        print "Service called"
        return resp2.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp2.success

#~ if __name__ == "__main__":
x= str(sys.argv[1])        
print call_url_retriever_server(x)       
	
