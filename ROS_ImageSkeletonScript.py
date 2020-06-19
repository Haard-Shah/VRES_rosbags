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
	def __init__(self):
		# VARIABLES
		self.speed = 0
		self.steering_angle = 0
		self.bridge = CvBridge()

		# ROS PUBLISHERS
		self.ackermannCommandPubliser = rospy.Publisher('/nightrider/ackermann_cmd/autonomous', AckermannDriveStamped, queue_size=10)

        # ROS SUBSCRIBERS
		self.imageSubscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.Image_Callback)

		# GET ROS PARAMETERS - might want to use these to limit your outputted speed and steering angle
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

			##############################################
			######## DO YOUR IMAGE PROCESSING HERE #######
			### OUTPUT LINEAR SPEED AND STEERING ANGLE ###
			##############################################
			
			# store linear speed and steering angle in message and publish
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



if __name__ == '__main__':
	followBlob = FollowBlob_ROS()

	# ROS NODE INIT
	rospy.init_node('blob_follow_demo')
	rospy.loginfo("Follow Demo Node Launched")

	try:
		# Had to use while not rospy.is_shutdown instead of rospy spin so imshow can be called
		while not rospy.is_shutdown():
			# Get and Show processed images if you wish image
			pass

	except Exception as e:
		raise e




