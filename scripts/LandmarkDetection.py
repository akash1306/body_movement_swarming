#!/usr/bin/env python3

# Landmarks

# 0. nose                   17. left_pinky
# 1. left_eye_inner         18. right_pinky
# 2. left_eve               19. left_index
# 3. left_eye_outer         20. right_index
# 4. right_eye_inner        21. left_thumb
# 5. right_eye              22. right_thumb
# 6. right_eye_outer        23. left_hip
# 7. left_ear               24. right_hip
# 8. right_ear              25. left_knee
# 9. mouth_left             26. right_knee
# 10. mouth_right           27. left_ankle
# 11. left_shoulder         28. right_ankle
# 12. right_shoulder        29. left_heel
# 13. left_elbow            30. right_heel
# 14. right_elbow           31. left_foot index
# 15. left_wrist            32. right_foot_index
# 16. right_wrist

from pickle import NONE
import rospy
import roslib

from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Float64Stamped
from mrs_msgs.msg import VelocityReferenceStamped
from mrs_msgs.msg import ReferenceStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from body_movement_swarming.msg import landmark

import math
import array
import numpy as np
import time
import os
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

import mediapipe as mp


# ROS Messages
from sensor_msgs.msg import CompressedImage, Image

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

class LandmarkDetectionClass(object):

    def __init__(self):

        self.br = CvBridge()
        self.image = NONE
        self.previous_sequence = -1
        self.total_frames = 0.0
        self.succ_frames = 0.0
        # self.cap = cv2.VideoCapture(0)

        self.landmarkpub = rospy.Publisher('/landmarkCoord' , landmark, \
                                            queue_size=10)
        self.subscriber = rospy.Subscriber("/uav1/rs_d435/color/image_raw", Image, \
                                            self.Callback)
        self.image_pub = rospy.Publisher("/mediapipe/image_raw", \
                                                Image, queue_size=10)
        # self.timer = rospy.Timer(rospy.Duration(0.03), self.TimerCallback)

        self.landmarkcoords = landmark()
        self.landmarkcoords.x = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.y = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.vis = array.array('f',(0 for f in range(0,33)))

        rospy.loginfo("Landmark Detector Initialized")

    def Callback(self, ros_data, image_encoding='bgr8'):
  

        try:
            self.image_header = ros_data.header
            self.image = self.br.imgmsg_to_cv2(ros_data, image_encoding)
            
        except CvBridgeError as e:
            if "[16UC1] is not a color format" in str(e):
                raise CvBridgeError(
                    "You may be trying to use a Image method " +
                    "(Subscriber, Publisher, conversion) on a depth image" +
                    " message. Original exception: " + str(e))
            raise e



    def PoseEstimator(self, pose):

        # success, self.image = self.cap.read()
        # self.image.flags.writeable = False
        imagefiller = self.image
        current_image_time = self.image_header.stamp
        imageRGB = cv2.cvtColor(imagefiller, cv2.COLOR_BGR2RGB)
        results = pose.process(imageRGB)
        imageBGR = cv2.cvtColor(imageRGB, cv2.COLOR_RGB2BGR)
        if self.image_header.seq == self.previous_sequence:
            return
        self.previous_sequence = self.image_header.seq

        if results.pose_landmarks:

            i=0
            self.succ_frames += 1.0
            for landname in mp_pose.PoseLandmark:
                self.landmarkcoords.name.append(str(landname))

                self.landmarkcoords.x[i] = results.pose_landmarks.landmark[landname].x 
                self.landmarkcoords.y[i] = results.pose_landmarks.landmark[landname].y
                self.landmarkcoords.vis[i] = results.pose_landmarks.landmark[landname].visibility 
                i+=1
            
            self.landmarkcoords.header.frame_id = "Human"
            self.landmarkcoords.header.stamp = current_image_time

            mp_drawing.draw_landmarks(
                imageBGR,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec= \
                            mp_drawing_styles.get_default_pose_landmarks_style())
            # cv2.imshow("Mediapipe Pose", self.image)
            image_2BGR = cv2.cvtColor(imageBGR, cv2.COLOR_RGB2BGR)
            landnmark_img =  self.br.cv2_to_imgmsg(image_2BGR, 'rgb8')
            self.image_pub.publish(landnmark_img)
            self.landmarkpub.publish(self.landmarkcoords)
            self.landmarkcoords.name = []

        # landnmark_img, imageRGB, imageBGR, image_2BGR = NONE


        



def main():
    rospy.init_node('Landmark_Detector', anonymous= True)
    rate = rospy.Rate(50)
    landmarkObject = LandmarkDetectionClass()

    # while not rospy.is_shutdown():
    #     rospy.loginfo_once("Entering ROS Loop")

    #     rate.sleep()
    # #rospy.sleep(1)
    # landmarkObject.calculateLandmarks()
    
    with mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        enable_segmentation=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.6) as pose:

        while not rospy.is_shutdown():
            if landmarkObject.image==NONE:
                continue
        
            rospy.loginfo_once("Entering while")
            landmarkObject.total_frames +=1.0
            landmarkObject.PoseEstimator(pose)
            print (landmarkObject.succ_frames / landmarkObject.total_frames)


            if cv2.waitKey(5) & 0xFF == 27:
                    break
            rate.sleep()



    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down Landmark Detector Node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



