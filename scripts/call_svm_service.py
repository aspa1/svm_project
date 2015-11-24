#!/usr/bin/env python

from svm_project.srv import *
import rospy
import sys


p = '/samples/positives' 				#positive_samples directory
n = '/samples/negatives'				#negative_samples directory
name='/test.png'						#name of the image to be tested 

def call_image_receiver_server(p, n):
    rospy.wait_for_service('image_receiver')
    try:
        service_handle = rospy.ServiceProxy('image_receiver', trainSvmSrv)
        resp1 = service_handle(p, n)
        print "Image receiver service called"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp1.success
        
print call_image_receiver_server(p,n)

def call_url_retriever_server():
    rospy.wait_for_service('url_retriever')
    try:
        service_handle2 = rospy.ServiceProxy('url_retriever', urlRetrieverSrv)
        resp2 = service_handle2(name)
        print "Url retriever service called"
        return resp2.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp2.success

     
print call_url_retriever_server()       
	
