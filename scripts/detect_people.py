#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

from rclpy.duration import Duration
from ultralytics import YOLO

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
		new_marker_topic = "/new_people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.scan = None
		self.new_marker_id = 0

		self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)
		self.marker_new_pub = self.create_publisher(Marker, new_marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []
		self.markers = []
		self.cluster_centers = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.get_logger().info(f"Running inference on image...")

			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue

				self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0]

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))

			cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)
def pointcloud_callback(self, data):
        if not self.rings:
            return

        # get point cloud attributes
        height = data.height
        width = data.width

        # 1. OPTIMIZACIJA: Preberi točke enkrat, ne v zanki (da ne "šteka")
        a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
        a = a.reshape((height, width, 3))

        # iterate over ring coordinates
        for x, y in self.rings:
            
            # Preveri meje (varnost)
            if x >= width or y >= height:
                continue

            # read 3D coordinates from point cloud
            d = a[y, x, :]

            # Preveri, če je točka sploh veljavna (da ni NaN)
            if np.isnan(d[0]) or np.isinf(d[0]):
                continue

            # Točka v frame-u kamere (v simulatorju je to običajno oakd_rgb_camera_optical_frame)
            point_in_cam_frame = PointStamped()
            point_in_cam_frame.header.frame_id = data.header.frame_id
            point_in_cam_frame.header.stamp = data.header.stamp

            point_in_cam_frame.point.x = float(d[0])
            point_in_cam_frame.point.y = float(d[1])
            point_in_cam_frame.point.z = float(d[2])

            time_now = rclpy.time.Time() # Uporabi Time(0) ali Time() za najnovejšo transformacijo
            timeout = Duration(seconds=0.1)

            try:
                # 2. KLJUČNI POPRAVEK: Transformacija iz frame-a podatkov (kamere) v MAPO
                trans = self.tf_buffer.lookup_transform("map", data.header.frame_id, data.header.stamp, timeout)
                
                # Transformacija točke
                point_in_map_frame = tfg.do_transform_point(point_in_cam_frame, trans)

                # 3. Ustvari marker (ID se mora povečevati, da ne brišeš starih)
                marker_in_map_frame = self.create_marker(point_in_map_frame, self.new_marker_id)

                # Publish
                self.marker_new_pub.publish(marker_in_map_frame)
                self.new_marker_id += 1
                
                self.get_logger().info(f"Marker objavljen na: {point_in_map_frame.point.x:.2f}, {point_in_map_frame.point.y:.2f}")

            except TransformException as te:
                self.get_logger().info(f"Cound not get the transform: {te}")
				

	# def merge_clusters(self):
	# 	centers = []
	# 	for p in self.markers:
	# 		p = p.pose.position
	# 		merged = False
	# 		for i, c in enumerate(centers):
	# 			if np.linalg.norm(np.array(p) - np.array(c)) <= 0.8:
	# 				centers[i] = ((np.array(c) + np.array(p)) / 2.0).tolist()
	# 				merged = True
	# 				break
	# 		if not merged:
	# 			centers.append(p)
	# 	self.cluster_centers = centers

	def _normalize_vector(self, vec):
		norm = np.linalg.norm(vec)
		if norm == 0.0:
			return (0.0, 0.0, 0.0)
		return (float(vec[0] / norm), float(vec[1] / norm), float(vec[2] / norm))

	def _rotate_vector_by_quaternion(self, vec, q):
		x = float(q.x)
		y = float(q.y)
		z = float(q.z)
		w = float(q.w)

		# Rotate a vector by a unit quaternion via the equivalent rotation matrix.
		r00 = 1.0 - 2.0 * (y * y + z * z)
		r01 = 2.0 * (x * y - z * w)
		r02 = 2.0 * (x * z + y * w)
		r10 = 2.0 * (x * y + z * w)
		r11 = 1.0 - 2.0 * (x * x + z * z)
		r12 = 2.0 * (y * z - x * w)
		r20 = 2.0 * (x * z - y * w)
		r21 = 2.0 * (y * z + x * w)
		r22 = 1.0 - 2.0 * (x * x + y * y)

		vx, vy, vz = vec
		return (
			float(r00 * vx + r01 * vy + r02 * vz),
			float(r10 * vx + r11 * vy + r12 * vz),
			float(r20 * vx + r21 * vy + r22 * vz),
		)

	def create_marker(self, point_stamped, marker_id, normal=None, lifetime=30.0):
		"""You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
		marker = Marker()

		marker.header = point_stamped.header

		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.id = marker_id

		# Set the scale of the marker
		scale = 0.1
		marker.scale.x = scale
		marker.scale.y = scale
		marker.scale.z = scale

		# Set the color
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# Set the pose of the marker
		marker.pose.position.x = point_stamped.point.x
		marker.pose.position.y = point_stamped.point.y
		marker.pose.position.z = point_stamped.point.z

		# Reuse orientation fields to carry the estimated face normal vector.
		if normal is not None:
			marker.pose.orientation.x = float(normal[0])
			marker.pose.orientation.y = float(normal[1])
			marker.pose.orientation.z = float(normal[2])
			marker.pose.orientation.w = 0.0

#		marker.lifetime = 0

		return marker


def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()