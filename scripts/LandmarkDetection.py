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
from cv_bridge import CvBridge

import mediapipe as mp


# ROS Messages
from sensor_msgs.msg import CompressedImage, Image

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

class LandmarkDetectionClass(object):

    def __init__(self):
        self.landmarkpub = rospy.Publisher('/landmarkCoord' , landmark, \
                                            queue_size=10)
        self.subscriber = rospy.Subscriber("/uav7/cam_out", Image, \
                                            self.callback,  queue_size = 1)
        self.image_pub = rospy.Publisher("/output/image_raw", Image, \
                                            queue_size=10)

        self.landmarkcoords = landmark()
        self.landmarkcoords.x = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.y = array.array('f',(0 for f in range(0,33)))
        self.landmarkcoords.vis = array.array('f',(0 for f in range(0,33)))

        rospy.loginfo("Landmark Detector Initialized")

    def callback(self, ros_data):
        i=0
        



def main():
    rospy.init_node('Landmark_Detector', anonymous= True)
    rate = rospy.Rate(50)
    landmarkObject = LandmarkDetectionClass()

    #rospy.sleep(1)
    # landmarkObject.calculateLandmarks()

    rospy.spin()

if __name__ == '__main__':
    main()



