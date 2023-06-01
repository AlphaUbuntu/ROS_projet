#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import cv2
#from enum import Enum
from std_msgs.msg import UInt8
#from sensor_msgs.msg import Image, CompressedImage
#from cv_bridge import CvBridge, CvBridgeError

class DetectSign():
    def __init__(self):
        self.sift = cv2.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        #dir_path = dir_path.replace('turtlebot3_autorace_detect/nodes', 'turtlebot3_autorace_detect/')
        #dir_path += 'images/'

        self.img_construction  = cv2.imread(dir_path + 'Stop.jpg',0)
        self.kp_construction, self.des_construction  = self.sift.detectAndCompute(self.img_construction, None)
      
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        #cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        
        MIN_MATCH_COUNT = 8 #9
        MIN_MSE_DECISION = 50000
        
        cap=cv2.VideoCapture(0)
        while true:
        
            while not rospy.is_shutdown():
                ret,frame=cap.read()
                if not ret:
                    rospy.logerr("Echec de la lecture de l'image")
                    break
                
                
                    # find the keypoints and descriptors with SIFT
                    kp1, des1 = self.sift.detectAndCompute(frame,None)
        
                    matches_construction = self.flann.knnMatch(des1,self.des_construction,k=2)
                  
                    image_out_num = 1
        
                    good_construction = []
                    for m,n in matches_construction:
                        if m.distance < 0.7*n.distance:
                        	good_construction.append(m)
                    if len(good_construction)>MIN_MATCH_COUNT:
                        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_construction ]).reshape(-1,1,2)
                        dst_pts = np.float32([self.kp_construction[m.trainIdx].pt for m in good_construction]).reshape(-1,1,2)
        
                        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                        matches_construction = mask.ravel().tolist()
        
                        mse = self.fnCalcMSE(src_pts, dst_pts)
                        if mse < MIN_MSE_DECISION:
                            #msg_sign = UInt8()
                            #msg_sign.data = self.TrafficSign.construction.value
        
                            #self.pub_traffic_sign.publish(msg_sign)
        
                            rospy.loginfo("Panneau détecté")
                            image_out_num = 2
                            rospy.sleep(0.5)