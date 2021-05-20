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
from actionlib_msgs.msg import GoalStatusArray, GoalID
from nav_msgs.msg import Odometry
import tf

import sys, select, termios, tty
import numpy as np

import time


class Robot_op:

	def __init__(self):
		self.name = ""
		self.detecting = False
		self.moving = False
		self.tasks = []
		self.objects = []
		self.objects_detected = []
		self.object_x = None
		self.object_y = None

		self.nav_goal_publisher = rospy.Publisher('move_base_simple/goal', Pose, queue_size = 1)
		self.nav_goal_listener = rospy.Subscriber('move_base/status', GoalStatusArray, self.Spin)
		self.position_listener = rospy.Subscriber('odom/ground_truth', Odometry, self.PrepareToStop)
		self.cancel_publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

		self.cmd_listener = rospy.Subscriber("taskChannel", CMD, self.ProcessCMD)

		self.od_target_publisher = rospy.Publisher('object_to_detect', Int8, queue_size=1)
		self.ack_listener = rospy.Subscriber("object_destination", ObjectDetectedMSG, self.Continue)

		

		rospy.spin()

	def PrepareToStop(self, data):
		if self.object_x and self.object_y and self.moving and np.sqrt(np.square(self.object_x-data.pose.pose.position.x)+np.square(self.object_y-data.pose.pose.position.y)) < 0.5:
			cancel_msg = GoalID()
			cancel_msg.id = ""
			cancel_msg.stamp = rospy.Time.now()
			self.cancel_publisher.publish(cancel_msg)
			self.moving = False
			self.object_x = None
			self.object_y = None

	def Spin(self, data):
		if data.status_list and (data.status_list[-1].status == 3 or data.status_list[-1].status == 2) and self.tasks and not self.detecting:
			if not self.objects:
				self.ExecuteCMD(self.tasks[1])
				del self.tasks[:]
				del self.objects[:]
				del self.objects_detected[:]
			else:
				self.Publish_to_object_detection(self.objects[0])
			


	def ProcessCMD(self, data):
		self.tasks.append(data)
		if len(self.tasks) == 2:
			self.ExecuteCMD(self.tasks[0])
		else:
			self.objects = list(data.obj_type)


	def Continue(self, data):
		if self.objects:
			if not data.class_id in self.objects_detected:
				self.objects_detected.append(data.class_id)
				self.objects.remove(data.class_id)
				self.ExecuteCMD(data)
		


	def ExecuteCMD(self, data):

		# MOVE TO GOAL

		body_pose = Pose()

		body_pose.header.frame_id="map"

		if self.detecting:
			self.object_x = data.goal_x
			self.object_y = data.goal_y

		body_pose.pose.position.x = data.goal_x
		body_pose.pose.position.y = data.goal_y


		quaternion = tf.transformations.quaternion_from_euler(0,0,0)
		body_pose.pose.orientation.x = quaternion[0]
		body_pose.pose.orientation.y = quaternion[1]
		body_pose.pose.orientation.z = quaternion[2]
		body_pose.pose.orientation.w = quaternion[3]

		self.nav_goal_publisher.publish(body_pose)

		self.detecting = False
		self.moving = True



	# SEND OBJECT TO OBJECT DETECTION
	def Publish_to_object_detection(self, data):
		object_to_detect = Int8()

		object_to_detect.data = data
		self.detecting = True
		 
		self.od_target_publisher.publish(object_to_detect)

		
	
if __name__ == "__main__":
	rospy.init_node('robot_op')
	robot_op = Robot_op()
