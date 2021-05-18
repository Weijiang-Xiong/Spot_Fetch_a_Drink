#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('robot_op')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import PoseStamped as Pose
from move_base_msgs.msg import MoveBaseGoal as NavGoal
from spot_audio_command.msg import MessageCMD as CMD
from image_processing.msg import ObjectDetectedMSG
from std_msgs.msg import Int8
import tf

import sys, select, termios, tty
import numpy as np

import time


class Robot_op:

	def __init__(self):
		self.name = ""
		self.tasks = []
		self.objects = []
		self.objects_detected = []

		self.nav_goal_publisher = rospy.Publisher('move_base_simple/goal', Pose, queue_size = 1)
		self.cmd_listener = rospy.Subscriber("taskChannel", CMD, self.ProcessCMD)

		self.od_target_publisher = rospy.Publisher('object_to_detect', Int8, queue_size=1)
		self.ack_listener = rospy.Subscriber("object_destination", ObjectDetectedMSG, self.Continue)

		rospy.spin()


	def ProcessCMD(self, data):
		self.tasks.append(data)
		if len(self.tasks) == 2:
			self.ExecuteCMD(self.tasks[0])
		else:
			self.objects = list(data.obj_type)


	def Continue(self, data):
		if not data.class_id in self.objects_detected:
			self.objects_detected.append(data.class_id)
			self.objects.remove(data.class_id)
			if not self.objects:
				self.ExecuteCMD(self.tasks[1])


	def ExecuteCMD(self, data):

		# MOVE TO GOAL

		body_pose = Pose()

		body_pose.header.frame_id="map"

		body_pose.pose.position.x = data.goal_x
		body_pose.pose.position.y = data.goal_y

		quaternion = tf.transformations.quaternion_from_euler(0,0,0)
		body_pose.pose.orientation.x = quaternion[0]
		body_pose.pose.orientation.y = quaternion[1]
		body_pose.pose.orientation.z = quaternion[2]
		body_pose.pose.orientation.w = quaternion[3]

		self.nav_goal_publisher.publish(body_pose)
		if self.objects:
			self.Publish_to_object_detection(self.objects[0])
		else:
			del self.tasks[:]
			del self.objects[:]
			del self.objects_detected[:]


	# SEND OBJECT TO OBJECT DETECTION
	def Publish_to_object_detection(self, data):
		object_to_detect = Int8()

		object_to_detect.data = data

		self.od_target_publisher.publish(object_to_detect)

		
	
if __name__ == "__main__":
	rospy.init_node('robot_op')
	robot_op = Robot_op()
