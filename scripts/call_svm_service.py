#!/usr/bin/env python

from svm_project.srv import *
import rospy
import sys


#~ p = '/samples/positives' 				#positive_samples directory
#~ n = '/samples/negatives'				#negative_samples directory
#~ name='/out.jpg'							#name of the image to be tested 

def call_image_receiver_server():
    rospy.wait_for_service('image_receiver')
    try:
        service_handle = rospy.ServiceProxy('image_receiver', trainSvmSrv)
        resp1 = service_handle(p, n)
        print "Image receiver service called"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp1.success
        

def call_uri_retriever_server():
    rospy.wait_for_service('uri_retriever')
    try:
        service_handle2 = rospy.ServiceProxy('uri_retriever', urlRetrieverSrv)
        resp2 = service_handle2(imagePath)
        print "Uri retriever service called"
        return resp2.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return resp2.success

def usage():
	#~ print "Requesting the folder path for positives,negatives and an image uri"
    return "%s [p n imagePath]"%sys.argv[0]
    #~ return "%s [x y]"%sys.argv[0]
     

if __name__ == "__main__":
    if len(sys.argv) == 4:
        p = str(sys.argv[1])
        n = str(sys.argv[2])
        imagePath= str(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
        
	print call_image_receiver_server()
	print call_uri_retriever_server()      
	
