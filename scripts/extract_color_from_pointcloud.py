#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import struct

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.scan = None


		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)


		self.faces = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def pointcloud_callback(self, data):

		# get point cloud attributes
		height = data.height
		width = data.width
		point_step = data.point_step
		row_step = data.row_step		


		# get 3-channel representation of the poitn cloud in numpy format
		a = pc2.read_points_numpy(data, field_names= ("x", "y", "z", "rgb"))

		a = a.reshape((height,width,4))

		print(a[150,150, :])

		print('color as a byte list',list(struct.pack("!f", a[150,150, -1])))



def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()