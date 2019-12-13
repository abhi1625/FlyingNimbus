import argparse
import numpy as np
import pandas as pd
import cv2
import os
import copy


class Fusion:
	def __init__(self):
		self.corner_sub = rospy.Subscriber("/corners", Pose, self.corners_cb)
		self.pose_pub = rospy.Publisher("/relative_pose", String, queue_size = 1)
		self.window_image_pub = rospy.Publisher("/window_img",Image, queue_size = 1)
		self.twist_obj = Twist()
		self.window_pose = Pose()
		self.bridge = CvBridge()

	
	def corners_cb(self):
