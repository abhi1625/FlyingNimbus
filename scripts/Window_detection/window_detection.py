#!/usr/bin/env python
import time
import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Pose
from cv_bridge import CvBridge, CvBridgeError
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
	sys.path.remove('/opt/ros/kinetic/share/opencv3/')
except:
	pass

import cv2
import numpy as np
import math
import copy
# from GMM.test_data import *

class video_stream:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.img_callback)
		self.window_detection = Window_detection()
		rospack = rospkg.RosPack()
		# pkg_path = rospack.get_path('noob_quaternions')

		#print(pkg_path)
		# weights_path = pkg_path+'/src/drone-course/Window_detection/GMM/training_params/window_weights_4.npy'
		# params_path = pkg_path+'/src/drone-course/Window_detection/GMM/training_params/gaussian_params_4.npy'
		# self.n, self.K, self.weights, self.params = loadparamsGMM(weights_path, params_path)
		self.image_cv = None

	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			# processed_img = preprocess_img(cv_image)
			mask = self.window_detection.segment_window(cv_image)
			self.window_detection.detection_hough_lines(cv_image,mask)

		
	
class Window_detection:
	def __init__(self):
		# self.data_path = '/home/prgumd/Desktop/pink_window.mp4'
		self.pose_pub = rospy.Publisher("/relative_pose", Pose, queue_size = 1)
		self.state_pub = rospy.Publisher("/vision_status", String, queue_size = 1)
		self.window_image_pub = rospy.Publisher("/window_img",Image, queue_size = 1)
		self.twist_obj = Twist()
		self.window_pose = Pose()
		self.bridge = CvBridge()
		self.resize_factor = 2
		################################################
		#moving mean filter
		self.center = np.array([0,0])
		self.centerpoint = np.array([0,0])
		self.count = 0
		self.top_left_corner = np.array([0,0])
		self.top_right_corner = np.array([0,0])
		self.bottom_left_corner = np.array([0,0])
		self.bottom_right_corner = np.array([0,0])
		self.top_left = np.array([0,0])
		self.top_right = np.array([0,0])
		self.bottom_left = np.array([0,0])
		self.bottom_right = np.array([0,0])
		self.height_offset = 1.0
		self.sliding_window_len = 3


	def segment_window(self,cv_image):
		# window param night light 100%
		thresh_r_min=187
		thresh_g_min=139
		thresh_b_min=49
		thresh_r_max=255
		thresh_g_max=255
		thresh_b_max=120

		# window param day 8:30 a.m. light 100%
		#thresh_r_min=250
		#thresh_g_min=163
		#thresh_b_min=90
		#thresh_r_max=255
		#thresh_g_max=255
		#thresh_b_max=167

		mask = cv2.inRange(cv_image,np.array([thresh_b_min, thresh_g_min, thresh_r_min]),np.array([thresh_b_max, thresh_g_max, thresh_r_max]))

		return mask

	def get_all_corners(self,img,img_orig,thresh = 1):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# gray = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)
		gray = cv2.medianBlur(gray,5)

		# edges = cv2.Canny(gray,50,150,apertureSize = 3)
		edges = gray
		# print("canny type", np.shape(edges))
		# cv2.imshow('image',edges)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		# closing to fill unwanted small gaps
		closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
		
		closing = cv2.GaussianBlur(closing, (7,7), cv2.BORDER_DEFAULT)
		corners = cv2.goodFeaturesToTrack(closing, 111, 0.1, 30)

		return np.squeeze(corners)

	def get_outer_window_corner_points(self, corner_pts, img):
		distances = corner_pts[:,0]**2 + corner_pts[:,1]**2
		h,w = img.shape[:2]
		# print("corner_pts = ",corner_pts)
		left_pt = np.argmin(distances)
		left_top_corner = corner_pts[left_pt,:]
		# print("")

		distances = (corner_pts[:,0]-w)**2 + (corner_pts[:,1])**2
		right_top_corner = corner_pts[np.argmin(distances),:]

		bottom_most_pt = np.argmax(corner_pts[:,1])
		# print("bottom_most_pt = ", corner_pts[bottom_most_pt,:])
		# corner_pts = np.delete(corner_pts,bottom_most_pt,0)


		distances = (corner_pts[:,0]-w)**2 + (corner_pts[:,1]-h)**2
		right_bottom_corner = corner_pts[np.argmin(distances),:]

		distances = corner_pts[:,0]**2 + (corner_pts[:,1]-h)**2
		left_bottom_corner = corner_pts[np.argmin(distances),:]

		imgPoints = np.array([[left_top_corner],
							  [right_top_corner],
							  [right_bottom_corner],
							  [left_bottom_corner]],dtype=np.int32)
		imgPoints = np.squeeze(imgPoints)

		return imgPoints

	def pnp(self, imgPoints,img):
		h,w = img.shape[:2]
		# World coordinates using window measurement in world
		objPoints = np.array([[0,0,0],
							 [84,0,0],
							 [81,43,0],
							 [3,43,0]], dtype=np.float64)

		# Camera K matrix(intrinsic params)
		camMatrix = np.array([[689.632, 0 , 664.887],[0, 687.574, 376.0835],[0,0,1]],dtype=np.float32)

		#distortion coefficients 
		distCoeffs = np.array([0,0,0,0,0],dtype=np.float64)

		_, rotVec, transVec = cv2.solvePnP(objPoints,imgPoints, camMatrix, distCoeffs)

		#print("rotation_vec: ",rotVec)
		#print("trans_vec: ", transVec)

		# Verification by reporjecting points using rotation and 
		# translation computed above
		reprojPoints,_ = cv2.projectPoints(objPoints, rotVec, transVec, camMatrix, distCoeffs)
		reprojPoints = np.squeeze(reprojPoints)
		# print(reprojPoints.shape)

		for i in range(4):
		        pt = reprojPoints[i]
		        imgpt = imgPoints[i]
		        cv2.circle(img, (int(pt[0]), int(pt[1])),5,[255,0,0],-1)
		        # cv2.circle(img, (int(imgpt[0]),int(imgpt[1])),5,[0,255,0],-1)
		# cv2.imshow('image',img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

		return rotVec,transVec



	def detection_hough_lines(self,original_img, mask):
		# img = cv2.imread(self.original_image)
		# mask = cv2.imread(self.image_path,0)
		# print("orig size",original_img.shape)
		# cv2.imshow("mask", mask)
		# cv2.waitKey(1)
		n_rows = int(original_img.shape[0]/self.resize_factor)
		n_cols = int(original_img.shape[1]/self.resize_factor)	
		img = cv2.resize(original_img, (n_cols, n_rows))
		mask = cv2.resize(mask, (n_cols, n_rows))

		gray = cv2.GaussianBlur(mask, (3,3), cv2.BORDER_DEFAULT)
		# edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
		minLength = 200
		maxLineGap = 80

		lines = cv2.HoughLinesP(gray,10, np.pi/180, 80, minLength, maxLineGap)
		# lines = cv2.HoughLines(gray, 1, np.pi/180, 10, None, 0, 0 )
		# print lines.shape

		# input('as')
		houghlines = np.zeros_like(gray)
		houghlines = np.dstack((houghlines,houghlines,houghlines))
		# print("houghlines shape",houghlines.shape)
		try:
			for x1, y1, x2, y2 in lines[:,0,:]:
				cv2.line(houghlines, (x1, y1),(x2,y2), (0,255,0), 2)
		except:
			print('lines not found')
			pass

		# print ("imgPoints = ",imgPoints)
		houghlines = cv2.cvtColor(houghlines, cv2.COLOR_RGB2GRAY)
		# cv2.imshow("houghlines init",houghlines)
		# cv2.waitKey(1)
		kernel_lines = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		# closing to fill unwanted small gaps
		houghlines = cv2.morphologyEx(houghlines, cv2.MORPH_CLOSE, kernel_lines)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(9,9))
		houghlines  = cv2.dilate(houghlines,kernel,iterations = 1)
		houghlines = cv2.morphologyEx(houghlines, cv2.MORPH_CLOSE, kernel)
		houghlines_gray = cv2.medianBlur(houghlines,5)
		n_rows,n_cols,_ = original_img.shape
		houghlines_gray = cv2.resize(houghlines_gray, (n_cols, n_rows))
		# cv2.imshow("resized hough gray",houghlines_gray)
		# cv2.waitKey(1)

		##############################################################
		#find contours
		houghlines_gray, contours, hierarchy = cv2.findContours(houghlines_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		epsilon = 0.1*cv2.arcLength(contours[0], True)
		approx = cv2.approxPolyDP(contours[0], epsilon, True)
		if ((len(approx) >= 4)):
			points = np.squeeze(np.array(approx))
			if cv2.isContourConvex(points):
				corners = self.get_outer_window_corner_points(points, original_img)
				cv2.drawContours(original_img, [approx], 0, (0,0,0), 3)
				self.center = np.vstack([self.center, np.mean([self.top_left,self.top_right,self.bottom_right, self.bottom_left],axis=0)])
				self.top_left_corner = np.vstack([self.top_left_corner, corners[0]])
				self.top_right_corner = np.vstack([self.top_right_corner, corners[1]])
				self.bottom_left_corner = np.vstack([self.bottom_left_corner, corners[3]])
				self.bottom_right_corner = np.vstack([self.bottom_right_corner, corners[2]])

				if self.top_left_corner.shape[0] > self.sliding_window_len:
					self.top_left_corner = np.delete(self.top_left_corner,0,0)
					self.top_right_corner = np.delete(self.top_right_corner,0,0)
					self.bottom_left_corner = np.delete(self.bottom_left_corner,0,0)
					self.bottom_right_corner = np.delete(self.bottom_right_corner,0,0)
					
				if self.center.shape[0] > self.sliding_window_len:
					self.center = np.delete(self.center,0,0)

				# print(self.center.shape)
				self.centerpoint = np.mean(self.center,axis=0)
				self.top_left = np.mean(self.top_left_corner, axis=0)
				self.top_right = np.mean(self.top_right_corner, axis=0)
				self.bottom_left = np.mean(self.bottom_left_corner, axis=0)
				self.bottom_right = np.mean(self.bottom_right_corner, axis=0)

				self.goodCorners = np.array([self.top_left, self.top_right, self.bottom_right, self.bottom_left])
				rotVec,transVec = self.pnp(self.goodCorners,original_img)
				# end = time.time()
				# print("time for pnp ", end - start_time)
				#publish quads pose relative to window
				# quad_x = camera_frame z
				# quad_y = camera_frame -x
				# quad_z = camera_frame -y
				# quad_yaw(counterclockwise) = camera_frame yaw(clockwise) 
				# window coordinates = [0,0,0] , [84,0,0]
				#					   [3,43,0], [81,43,0]
				# window center coordinates = [42,21.5,0]
				self.window_pose.position.x = (transVec[2,0])/100 			#in m
				self.window_pose.position.y = -(transVec[0,0] + 41.0)/100 	#in m
				self.window_pose.position.z = -(transVec[1,0] + 21.5)/100  	#in m
				self.window_pose.orientation.z = float(-rotVec[1])
				# print("state x,y,z,yaw",self.twist_obj.linear.x,self.twist_obj.linear.y,self.twist_obj.linear.z,self.twist_obj.angular.z)
				print("x {}, y{}, z{}, yaw {}".format(self.window_pose.position.x,self.window_pose.position.y,self.window_pose.position.z,self.window_pose.orientation.z))
				self.pose_pub.publish(self.window_pose)
				self.state_pub.publish("active")
		else:
			self.state_pub.publish("inactive")
			
		# corner_pts = self.get_all_corners(houghlines, original_img)
		# imgPoints = self.get_outer_window_corner_points(corner_pts, original_img)
		# centerpoint = np.mean(imgPoints, axis=0)
		# print(corner_pts.shape)
		# input('as')
		cv2.circle(original_img, tuple((int(self.centerpoint[0]), int(self.centerpoint[1]))), 5, (0,0,0), -1)

		#corners
		cv2.circle(original_img, tuple((int(self.top_left[0]), int(self.top_left[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.top_right[0]), int(self.top_right[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.bottom_right[0]), int(self.bottom_right[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.bottom_left[0]), int(self.bottom_left[1]))), 5, (0,0,0), -1)


		# for i in range(imgPoints.shape[0]):
		# 	pt = imgPoints[i]
		# 	cv2.circle(original_img,tuple(pt),3,(0,0,255),-1)
		# except:
		# 	pass

		#cv2.imshow('corner',original_img)
		#cv2.waitKey(1)
		#cv2.imshow('frame',houghlines)
		# cv2.imshow('frame1',masked_img)
		#if cv2.waitKey(1) & 0xFF == ord('q'):
		#	cv2.destroyAllWindows()
		# masked_img = cv2.resize(img, (int(img.shape[1]/2), (int(img.shape[0]/2))))
		cv_image = self.bridge.cv2_to_imgmsg(original_img, "bgr8")
		self.window_image_pub.publish(cv_image)





def main():
	count = 0
	rospy.init_node('image_reader',anonymous=True)
	ob = video_stream()

	#rospy.init_node('image_reader', anonymous=True)
	rate = rospy.Rate(30)
	while(not rospy.is_shutdown()):
		# rospy.spin()
		# ob.processing()
		count += 1;
		rate.sleep()

if __name__ == '__main__':
	main()
