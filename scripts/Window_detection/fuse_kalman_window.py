import argparse
import numpy as np
import pandas as pd
import cv2
import os
import copy


class Fusion:
	def __init__(self):
		self.pose_pub = rospy.Subscriber("/relative_pose", Pose, queue_size = 1)
		self.state_pub = rospy.Publisher("/vision_status", String, queue_size = 1)
		self.window_image_pub = rospy.Publisher("/window_img",Image, queue_size = 1)
		self.twist_obj = Twist()
		self.window_pose = Pose()
		self.bridge = CvBridge()