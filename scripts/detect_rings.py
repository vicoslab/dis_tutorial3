#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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

from rclpy.duration import Duration

class detect_rings(Node):

    def __init__(self):
        super().__init__('detect_rings')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', ''),
        ])

        marker_topic = "/rings_marker"
        new_marker_topic = "/new_rings_marker"

        self.detection_color = (0,0,255)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.new_marker_id = 0

        self.ration_thr = 1.5
        self.center_thr = 5.0 # Malo povečano za boljšo toleranco v simulatorju

        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, marker_topic, qos_profile_sensor_data)
        self.marker_new_pub = self.create_publisher(Marker, new_marker_topic, qos_profile_sensor_data)

        self.rings = []
        self.markers = []

        self.get_logger().info(f"Node has been initialized! Will publish ring markers to {marker_topic}.")

    def rgb_callback(self, data):
        self.rings = []
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # Parametri za HoughCircles: omejitev velikosti (maxRadius)
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=50,
                param1=100,
                param2=35,     # Višja številka = bolj stroga detekcija
                minRadius=10,
                maxRadius=80,  # Omejitev, da ne prepozna prevelikih objektov
            )

            if circles is not None:
                circles = np.round(circles[0, :]).astype(int)
                for x, y, radius in circles:
                    self.get_logger().info(f"Obroč zaznan (Hough): {x}, {y}")
                    cv2.circle(cv_image, (x, y), radius, (0, 255, 0), 2)
                    cv2.circle(cv_image, (x, y) , 5, (0, 0, 255), -1)
                    self.rings.append((x, y))
            else:
                # Fallback na elipse, če Hough ne najde nič
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
                contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                elps = []
                for cnt in contours:
                    if len(cnt) >= 20:
                        ellipse = cv2.fitEllipse(cnt)
                        (x, y), (MA, ma), angle = ellipse
                        ration = MA / ma if ma > 0 else 0
                        if 1.0 < ration <= self.ration_thr:
                            elps.append(ellipse)

                for i in range(len(elps)):
                    for j in range(i + 1, len(elps)):
                        e1 = elps[i]
                        e2 = elps[j]
                        dist = np.sqrt((e1[0][0] - e2[0][0])**2 + (e1[0][1] - e2[0][1])**2)
                        if dist < self.center_thr:
                            cx, cy = int(e1[0][0]), int(e1[0][1])
                            self.get_logger().info(f"Obroč zaznan (Ellipse): {cx}, {cy}")
                            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
                            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                            self.rings.append((cx, cy))

            cv2.imshow("Detection Window", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def pointcloud_callback(self, data):
        if not self.rings:
            return

        try:
            # Preberemo numpy točke ENKRAT pred zanko (optimizacija)
            a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            a = a.reshape((data.height, data.width, 3))

            for x, y in self.rings:
                if x >= data.width or y >= data.height:
                    continue

                d = a[y, x, :]

                if np.isnan(d[0]) or np.isinf(d[0]):
                    continue

                # Točka v frame-u kamere
                point_in_cam_frame = PointStamped()
                point_in_cam_frame.header.frame_id = data.header.frame_id
                point_in_cam_frame.header.stamp = data.header.stamp      
                point_in_cam_frame.point.x = float(d[0])
                point_in_cam_frame.point.y = float(d[1])
                point_in_cam_frame.point.z = float(d[2])
                time_now = rclpy.time.Time()

                timeout = Duration(seconds=0.2)
                try:
                    # Transformacija v MAP frame
                    trans = self.tf_buffer.lookup_transform("map", data.header.frame_id, time_now, timeout)
                    point_in_map_frame = tfg.do_transform_point(point_in_cam_frame, trans)

                    # Ustvari marker v mapi
                    marker_in_map_frame = self.create_marker(point_in_map_frame, self.new_marker_id, 0.0)

                    # Objavi marker
                    self.marker_new_pub.publish(marker_in_map_frame)
                    self.new_marker_id += 1
                    
                    # Originalen marker publisher (kot si imel v kodi)
                    self.marker_pub.publish(marker_in_map_frame)
                    self.markers.append(marker_in_map_frame)

                except TransformException as te:
                    self.get_logger().warn(f"Could not get transform: {te}")

        except Exception as e:
            self.get_logger().error(f"Pointcloud error: {e}")

    def create_marker(self, point_stamped, marker_id, lifetime=30.0):
        marker = Marker()
        marker.header = point_stamped.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = marker_id

        scale = 0.15
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        marker.lifetime = Duration(seconds=lifetime).to_msg()
        return marker

def main():
    rclpy.init(args=None)
    node = detect_rings()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()