#!/usr/bin/env python

# IMPORT CLASSES
import cv2
import math
import rospy
import numpy as np
from cv_bridge import CvBridge

# IMPORT MESSAGE TYPES
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image


### CLASS TO HANDLE ALL ROS STUFF ###
class FollowBlob_ROS():
	# INITIALISATION
	def __init__(self, trackbar_window_name):
		# VARIABLES
		self.LIN_VEL_GAIN = 0.00005
		self.STEERING_GAIN = 2

		self.speed = 0
		self.steering_angle = 0

		self.trackbar_window_name = trackbar_window_name
		self.img_masked = np.zeros((20, 20))
		self.bridge = CvBridge()


		# ROS PUBLISHERS
		self.ackermannCommandPubliser = rospy.Publisher('/nightrider/ackermann_cmd/autonomous', AckermannDriveStamped, queue_size=10)

        # ROS SUBSCRIBERS
		self.imageSubscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.Image_Callback)

		# GET ROS PARAMETERS
		self.maxSteeringAngle = rospy.get_param('max_steering_angle', math.radians(45.0))
		self.maxForwardSpeed = rospy.get_param('max_foward_velocity', 2.0)
		self.maxReverseSpeed = rospy.get_param('max_reverse_velocity', -1.0)
		self.acceleration = rospy.get_param('acceleration', 0.5) # check nightrider config file for definition

		# SETUP ROS MESSAGES
		# Ackermann Drive Command
		self.ackermann_cmd = AckermannDriveStamped()
		self.ackermann_cmd.drive.steering_angle_velocity = 0.0 # see AckermannDriveStamped message for definition
		self.ackermann_cmd.drive.acceleration = self.acceleration # see AckermannDriveStamped message for definition
		self.ackermann_cmd.drive.jerk = 0 # see AckermannDriveStamped message for definition


	def Image_Callback(self, data):
		self.speed = 0

		try:
			# Attempt to convert ros image to cv2 8-bit bgr image
			img_bgr = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# Resize image
			img_bgr = cv2.resize(img_bgr, (480, 320))

			# Detect Blob Center
			blobCenter, contourArea, self.img_masked = self.DetectBlob(img_bgr)	
			maxArea = GetAreaThresholds(self.trackbar_window_name)[1]
			
			if blobCenter != None:
				# Determine range bearing - 
				# range is maxArea - actualArea so objects further away have a bigger _range value
				# using very simple linear mapping for bearing (I think FOV is 76 degrees)
				_range = (maxArea - contourArea) 
				_bearing = blobCenter[0] * (math.radians(38) - math.radians(-38)) / (img_bgr.shape[1]) + math.radians(-38);
				_bearing = -1 * _bearing # need to invert bearing
				# rospy.loginfo("Range: %0.2f, Bearing %0.2f"%(_range, _bearing*180.0/math.pi))

				# Proportional controller to get desired speed and steering angle - use min/max to cap to limits
				self.speed = min(self.maxForwardSpeed, max(_range * self.LIN_VEL_GAIN, self.maxReverseSpeed))
				self.steering_angle = min(self.maxSteeringAngle, max(_bearing * self.STEERING_GAIN, -1*self.maxSteeringAngle))
			
			# store data in message and publish
			self.ackermann_cmd.drive.steering_angle = self.steering_angle
			self.ackermann_cmd.drive.speed = self.speed
			self.ackermann_cmd.header.stamp = rospy.Time.now()
			self.ackermannCommandPubliser.publish(self.ackermann_cmd)

		except Exception as e:
			rospy.logwarn("In Blob Follow Demo Node Image_Callback Function. Error: " + str(e))

			self.speed = 0
			self.ackermann_cmd.drive.speed = self.speed
			self.ackermann_cmd.header.stamp = rospy.Time.now()
			self.ackermannCommandPubliser.publish(self.ackermann_cmd)


	def DetectBlob(self, img_bgr):
		# setup variables
		centerPoint = None
		largestContour = None
		largestContourArea = 0
		blobFound = False

		# Pre-process image using gaussian blur with 5x5 kernel and 0 standard deviation
		img_bgr = cv2.GaussianBlur(img_bgr, (5,5), 0)

		# Threshold the image using the HSV values
		img_bw, img_masked = self.ThresholdImage(img_bgr)

		# Many ways to do blob filtering in cv2. Going to use contour method
		_, contours, _ = cv2.findContours(img_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# Get Area Thresholds
		area_thresh = GetAreaThresholds(self.trackbar_window_name)

		# Filter areas and take largest contour greater than the minimum area
		for con in contours:
			area = cv2.contourArea(con)
			if (area > area_thresh[0] and area < area_thresh[1]) and (blobFound == False or area > cv2.contourArea(largestContour)):
				largestContour = con
				largestContourArea = area
				blobFound = True

		# Get centre point of contour
		if blobFound == True:
			m = cv2.moments(largestContour)
			cx = int(m["m10"] / m["m00"])
			cy = int(m["m01"] / m["m00"])

			centerPoint = [cx, cy]

		# Draw contour on the masked image in green
		if blobFound == True:
			cv2.drawContours(img_masked, [largestContour], 0, (0, 255, 0), 3)

		return centerPoint, largestContourArea, img_masked


	def ThresholdImage(self, img_bgr):
		# Convert bgr image to hsv image
		img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

		# Get HSV thresholds
		hue_thresh, sat_thresh, val_thresh = GetHSVThresholds(self.trackbar_window_name)

		# Due to wrap around of hue value in the HSV colour space need to check if min value is greater than max value
		if hue_thresh[0] > hue_thresh[1]:
			# Will need to do threshold from min to 180 and from 0 to maximum to get correct hue thresholded image

			# First do min to 180 hue threshold. Create numpy arrays for min and max hsv thresholds
			lower_thresh = np.array([hue_thresh[0], sat_thresh[0], val_thresh[0]])
			upper_thresh = np.array([180, sat_thresh[1], val_thresh[1]])

			# Threshold image
			img_bw1 = cv2.inRange(img_hsv, lower_thresh, upper_thresh)

			# First do min to 180 hue threshold. Create numpy arrays for min and max hsv thresholds
			lower_thresh = np.array([0, sat_thresh[0], val_thresh[0]])
			upper_thresh = np.array([hue_thresh[1], sat_thresh[1], val_thresh[1]])

			# Threshold image
			img_bw2 = cv2.inRange(img_hsv, lower_thresh, upper_thresh)

			# Add threshold images together using bitwise or
			img_bw = cv2.bitwise_or(img_bw1, img_bw2)

		else:
			# Simply need to threshold from hue min to hue max
			
			# Create numpy arrays for min and max hsv thresholds
			lower_thresh = np.array([hue_thresh[0], sat_thresh[0], val_thresh[0]])
			upper_thresh = np.array([hue_thresh[1], sat_thresh[1], val_thresh[1]])

			# Threshold image
			img_bw = cv2.inRange(img_hsv, lower_thresh, upper_thresh)

		# Apply img_bw (which is binary mask to bgr image). This masked image can be used to for viewing.
		img_masked = cv2.bitwise_and(img_bgr, img_bgr, mask=img_bw)

		# Return img_bw and img_masked
		return img_bw, img_masked


### OPEN CV FUNCTIONS THAT DISPLAY THINGS ###
def InitializeWindowsAndTrackbars(image_window_name, trackbar_window_name):
		# Create Named Window to which will show thresholded image
		cv2.namedWindow(image_window_name, cv2.WINDOW_NORMAL)
		cv2.moveWindow(image_window_name, 40,30)

		# Create Named Window for trackbars
		cv2.namedWindow(trackbar_window_name, cv2.WINDOW_NORMAL)
		cv2.moveWindow(trackbar_window_name, 40,30)

		# Create trackbars for hue, saturation and value thresholds
		cv2.createTrackbar('Hue Min', trackbar_window_name, 0, 180, DummyTrackbar_Callback)
		cv2.createTrackbar('Hue Max', trackbar_window_name, 0, 180, DummyTrackbar_Callback)

		cv2.createTrackbar('Sat Min', trackbar_window_name, 0, 255, DummyTrackbar_Callback)
		cv2.createTrackbar('Sat Max', trackbar_window_name, 0, 255, DummyTrackbar_Callback)

		cv2.createTrackbar('Val Min', trackbar_window_name, 0, 255, DummyTrackbar_Callback)
		cv2.createTrackbar('Val Max', trackbar_window_name, 0, 255, DummyTrackbar_Callback)

		# Set hsv trackbar initial values
		cv2.setTrackbarPos('Hue Min', trackbar_window_name, 165)
		cv2.setTrackbarPos('Hue Max', trackbar_window_name, 12)

		cv2.setTrackbarPos('Sat Min', trackbar_window_name, 65)
		cv2.setTrackbarPos('Sat Max', trackbar_window_name, 255)

		cv2.setTrackbarPos('Val Min', trackbar_window_name, 40)
		cv2.setTrackbarPos('Val Max', trackbar_window_name, 255)

		# Create and set trackbar for min and max blob size
		cv2.createTrackbar('Area Min', trackbar_window_name, 0, 20000, DummyTrackbar_Callback)
		cv2.createTrackbar('Area Max', trackbar_window_name, 0, 20000, DummyTrackbar_Callback)

		# Set area trackbar initial values
		cv2.setTrackbarPos('Area Min', trackbar_window_name, 500)
		cv2.setTrackbarPos('Area Max', trackbar_window_name, 20000)


def DummyTrackbar_Callback(val):
	pass


def GetHSVThresholds(window_name):
	hue_thresh = [0, 0]
	sat_thresh = [0, 0]
	val_thresh = [0, 0]

	hue_thresh[0] = cv2.getTrackbarPos('Hue Min', window_name)
	hue_thresh[1] = cv2.getTrackbarPos('Hue Max', window_name)

	sat_thresh[0] = cv2.getTrackbarPos('Sat Min', window_name)
	sat_thresh[1] = cv2.getTrackbarPos('Sat Max', window_name)

	val_thresh[0] = cv2.getTrackbarPos('Val Min', window_name)
	val_thresh[1] = cv2.getTrackbarPos('Val Max', window_name)

	return hue_thresh, sat_thresh, val_thresh


def GetAreaThresholds(window_name):
	area_thresh = [0, 0]

	area_thresh[0] = cv2.getTrackbarPos('Area Min', window_name)
	area_thresh[1] = cv2.getTrackbarPos('Area Max', window_name)

	return area_thresh


def ShowImage(window_name, img, cvWaitKeyVal = 1):
	cv2.imshow(window_name, img)
	cv2.waitKey(cvWaitKeyVal) 



if __name__ == '__main__':
	trackbar_window_name = "Trackbar Window"
	image_window_name = "Masked Image"
	InitializeWindowsAndTrackbars(image_window_name, trackbar_window_name)
	followBlob = FollowBlob_ROS(trackbar_window_name)

	# ROS NODE INIT
	rospy.init_node('blob_follow_demo')
	rospy.loginfo("BLOB Follow Demo Node Launched")

	try:
		# Had to use while not rospy.is_shutdown instead of rospy spin so imshow would get called.
		while not rospy.is_shutdown():
			# Get and Show masked image
			img_masked = followBlob.img_masked
			ShowImage(image_window_name, img_masked)

	except Exception as e:
		raise e




