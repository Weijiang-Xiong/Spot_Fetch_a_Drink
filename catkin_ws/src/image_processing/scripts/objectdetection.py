#!/usr/bin/env python3
import rospy
import math
import numpy as np
import traceback
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from image_processing.msg import ObjectDetectedMSG
from sensor_msgs.msg import  PointCloud2, Image
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
import tf2_ros, tf2_geometry_msgs

class ObjectDetectionServer:
	def __init__(self):
		self.darknet = None
		self.objectsCount = None
		self.pointcloud = None
		self.image = None
		self.location = None
		
		#trained objects
		self.class_list = ["beer", "coke", "extinguisher", "hammer", "screwdriver", "wrench", "person", "cylinder", "cube"]

		#object requested to detect (by speech)
		self.classRequested = None
		
		#message counter
		self.message_id = 0
		
		#center pixel coordinates
		self.default_x = 320
		self.default_y = 240
		
		# Set 0 for real time
		self.publishing_rate = 1

		self.latestTimeStamp = None


		
	# One callback to get xy coordinates and another to count the number of detected objects	
	def darknet_callback(self, msg):
		# "Store" message received.
		self.darknet = msg
		self.get_position()
		
	def darknet_counter_callback(self, msg):
		# "Store" message received.
		self.objectsCount = msg
		self.get_position()
		
	
	# Get pointcloud data
	def pointcloud_callback(self, msg):
		# "Store" the message received.
		self.pointcloud = msg
		self.get_position()
	
	# For testing	
	def image_callback(self, msg):
		# "Store" the message received.
		self.image = msg
		self.get_dimensions()
	
	# Speech/navigation callback
	def speech_callback(self, msg):
		# "Store" the message received.
		self.classRequested = msg.data
		self.get_position()

	# Get current position of the robot	
	def location_callback(self, msg):
		# "Store" the message received.
		self.location = msg.pose.pose.position
	
	# Convert xy to xyz and publish 	
	def get_position(self):
	
		self.x_loc = None
		self.y_loc = None
		self.z_loc = None
		
		if (self.darknet is not None) and (self.darknet.bounding_boxes is not None) and (self.pointcloud is not None) and (self.classRequested is not None):
			for item in self.darknet.bounding_boxes:
				if item.Class == self.class_list[self.classRequested]:
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
					
					self.convert_to_worldframe(np.array([self.x_loc, self.y_loc, self.z_loc]))	
					#rospy.sleep(self.publishing_rate)
					self.classRequested = None
					break
				else:
					self.convert_to_worldframe(None)
					
		else:

			self.convert_to_worldframe(None)
			pass		
			
	# For testing
	def get_dimensions(self):
		if self.image is not None: 
			pass
			##640 width 
			##480 height
			self.height = self.image.height
			self.width = self.image.width

	
	
	# Conversion to world frame
	def convert_to_worldframe(self, data):
		
		try:
			if data is None or self.location is None: 
				return

			target_frame = "base_link"
			source_frame = "camera_depth_optical_frame"
			
			# define a source point
			point_wrt_source = Point(data[0], data[1], data[2])
			
			transformation = tf_buffer.lookup_transform(target_frame,source_frame, rospy.Time(0), rospy.Duration(0.1))
			
			point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),transformation).point
			
			self.publish_destination(np.array([self.location.x + point_wrt_target.x, self.location.y + point_wrt_target.y, self.location.z + point_wrt_target.z, 
			point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]))


		
		except:
			traceback.print_exc()	
		
		
		
	
	# Publishing the data to topic /object_destination
	def publish_destination(self, data):
				
		#Custom message with detected object data
		msg = ObjectDetectedMSG()
		msg.message_id = self.message_id
		msg.header = Header()
		msg.header.frame_id = "map"
		msg.header.stamp = rospy.Time.now()
		


		if self.darknet.bounding_boxes is not None and  (self.latestTimeStamp is None or self.latestTimeStamp < msg.header.stamp): 
			# Publishing message to topic
			msg.class_id = self.classRequested
			msg.class_name = self.class_list[self.classRequested]
			
			msg.goal_x = data[0]
			msg.goal_y = data[1]
			msg.goal_z = data[2]
			
			msg.distance_from_robot_x = data[3]
			msg.distance_from_robot_y = data[4]
			msg.distance_from_robot_z = data[5]

			rospy.loginfo("Publishing data to {} /object_detection..".format(msg.message_id))	
			self.pub.publish(msg)
			self.message_id += 1
			self.latestTimeStamp = msg.header.stamp 
		


if __name__ == '__main__':
	try:	
		rospy.init_node('vision')
		#rospy.wait_for_service('spawn')
		server = ObjectDetectionServer()
		
		server.pub = rospy.Publisher('object_destination', ObjectDetectedMSG, queue_size=1)

		rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, server.darknet_callback)
		rospy.Subscriber('/darknet_ros/found_object', ObjectCount, server.darknet_counter_callback)
		rospy.Subscriber('/camera/depth/points', PointCloud2, server.pointcloud_callback)
		rospy.Subscriber('/odom/ground_truth', Odometry, server.location_callback)

		
		# Not used
		rospy.Subscriber('/camera/rgb/image_raw', Image, server.image_callback)
		
		## Add the topic from Navigation/speech when ready
		rospy.Subscriber('/object_to_detect', Int8, server.speech_callback, queue_size=1)
		
		# testing for transformations
		tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		listener = tf2_ros.TransformListener(tf_buffer)
		
		rospy.spin()
	
	except rospy.ROSInterruptException:
		pass
		
	
