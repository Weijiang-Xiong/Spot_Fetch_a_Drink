#!/usr/bin/env python
import os
import rospy
import math
import numpy as np
import message_filters
import traceback
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from image_processing.msg import ObjectDetectedMSG
from sensor_msgs.msg import  PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
import tf2_ros, tf2_geometry_msgs


class ObjectDetectionServer:
	def __init__(self):
		self.darknet = None
		self.objectsCount = None
		self.pointcloud = None
		self.image = None
		
		
		#trained objects
		self.class_list = ["cup", "beer", "coke", "extinguisher", "hammer", "wrench", "screwdriver", "person"]
		
		#object requested to detect (by speech), default "beer"
		self.classId = 0
		self.classRequested = self.class_list[self.classId]
		
		#message counter
		self.message_id = 0
		
		#center pixel coordinates
		self.default_x = 320
		self.default_y = 240
		
		# Set 0 for real time
		self.publishing_rate = 0
		
		
		
	# One callback to get xy coordinates and another to count the number of detected objects	
	def darknet_callback(self, msg):
		# "Store" message received.
		self.darknet = msg
		# Compute stuff.
		self.get_position()
		
	def darknet_counter_callback(self, msg):
		# "Store" message received.
		self.objectsCount = msg
		# Compute stuff.
		self.get_position()
		
		
	
	# Get pointcloud data
	def pointcloud_callback(self, msg):
		# "Store" the message received.
		self.pointcloud = msg
		# Compute stuff.
		self.get_position()
	
	# For testing	
	def image_callback(self, msg):
		# "Store" the message received.
		self.image = msg
		# Compute stuff.
		self.get_dimensions()
	
	#TODO Add speech/navigation callback
	def speech_callback(self, msg):
		# "Store" the message received.
		self.classRequested = msg
		# Compute stuff.
		self.process_request()
	
	# Convert xy to xyz and publish 	
	def get_position(self):
	
		
		# For clearing the screen 
		#clear = lambda: os.system('clear')
		#clear()
	
		if self.darknet is not None and self.darknet.bounding_boxes is not None and self.pointcloud is not None and self.classRequested is not None:
			
			for item in self.darknet.bounding_boxes:
				if item.Class == self.classRequested:
					x = math.trunc((item.xmin + item.xmax)/2) #width
					y = math.trunc((item.ymin + item.ymax)/2) #height
					
					# get the xyz at that location
					dtype_list = [(f.name, np.float32) for f in self.pointcloud.fields[0:3]]
					dtype_list.append(('rgb.r', np.float32))
					dtype_list.append(('rgb.g', np.float32))
					dtype_list.append(('rgb.b', np.float32))
					dtype_list.append(('rgb.a', np.float32))
					dtype_list.append(('rgb.x', np.float32))
					cloud_arr = np.frombuffer(self.pointcloud.data, dtype_list)
					cloud_arr = np.reshape(cloud_arr, (self.pointcloud.height, self.pointcloud.width))
					# this will get the x,y,z position at image[x][y]
					
					self.x_loc = cloud_arr[int(y)][int(x)][0]
					self.y_loc = cloud_arr[int(y)][int(x)][1]
					self.z_loc = cloud_arr[int(y)][int(x)][2]
					
					if self.objectsCount.count == 0:
						self.x_loc = 0
						self.y_loc = 0
						self.z_loc = 0
					
					#print(self.image.header.frame_id)
					self.convert_to_worldframe(np.array([self.x_loc, self.y_loc, self.z_loc]))	
					#rospy.loginfo("Publishing data to {} /object_detection..".format(self.message_id))	
					#rospy.sleep(self.publishing_rate)
					return
					
		else:
			rospy.loginfo("Waiting for data..")	
			#rospy.sleep(self.publishing_rate)
	
	
	# TODO Do preprocessing stuff with navigation data
	def process_request():
		if self.classRequested is not None: 
			pass
			
	# For testing
	def get_dimensions(self):
		if self.image is not None: 
			pass
			##640 width 
			##480 height
			self.height = self.image.height
			self.width = self.image.width

	
	def convert_to_worldframe(self, data):
		
		try:
			target_frame = "map"
			source_frame = "head_camera_depth_optical_frame"
			
			# define a source point
			point_wrt_source = Point(data[0], data[1], data[2])
			
			transformation = tf_buffer.lookup_transform(target_frame,source_frame, rospy.Time(0), rospy.Duration(0.1))
			
			point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),transformation).point
			
			self.publish_destination(np.array([point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]))
		
		except:
			traceback.print_exc()	
		
		
		
	
	# Publishing the data to topic /object_destination
	def publish_destination(self, data):
				
		#Custom message with detected object data
		msg = ObjectDetectedMSG()
		msg.message_id = self.message_id
		msg.header = Header()
		
		#TODO Add the requested class from navigation
		msg.class_id = self.classId 
		msg.class_name = self.classRequested
		
		msg.goal_x = data[0]
		msg.goal_y = data[1]
		msg.goal_z = data[2]
		
		self.pub.publish(msg)
		self.message_id += 1
		


if __name__ == '__main__':
	try:	
		rospy.init_node('listener')
		#rospy.wait_for_service('spawn')
		server = ObjectDetectionServer()
		
		server.pub = rospy.Publisher('object_destination', ObjectDetectedMSG, queue_size=0)

		rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, server.darknet_callback)
		rospy.Subscriber('/darknet_ros/found_object', ObjectCount, server.darknet_counter_callback)
		rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, server.pointcloud_callback)
		
		# Not used
		rospy.Subscriber('/head_camera/rgb/image_raw', Image, server.image_callback)
		
		## Add the topic from Navigation/speech when ready
		#rospy.Subscriber('/speechTOPIC', messagetypeTODO, server.speech_callback)
		
		
		
		
		# testing for transformations
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		listener = tf2_ros.TransformListener(tf_buffer)
		
		#self.listener = tf.TransformListener()
		
		
		rospy.spin()
	
	except rospy.ROSInterruptException:
		pass
		
	
