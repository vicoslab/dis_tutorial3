#!/usr/bin/env python3

import json
import math
import os
import threading
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_commander import RobotCommander, TaskResult


class PatrolPeopleCollector(Node):
    def __init__(self):
        super().__init__('patrol_people_collector')

        self.declare_parameter('cluster_radius_m', 0.25)
        self.declare_parameter('face_goal_offset_m', 0.5)
        self.declare_parameter('markers_output_file', '')

        self.cluster_radius_m = float(self.get_parameter('cluster_radius_m').value)
        self.face_goal_offset_m = float(self.get_parameter('face_goal_offset_m').value)

        # From poz.txt
        self.predefined_positions = [
            {
                'position': {'x': 0.9992520366512115, 'y': 5.099285747708961, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.06709634580279415, 'w': 0.9977465010612224},
            },
            {
                'position': {'x': -0.9625339498989449, 'y': 3.266840370555999, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9895548398428475, 'w': 0.14415692471607602},
            },
            {
                'position': {'x': -3.7034241420503897, 'y': 3.838940628120296, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9973686480486497, 'w': 0.07249675778687495},
            },
        ]

        # Output path
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_output_file = os.path.join(pkg_root, 'data', 'detected_people.json')
        configured_output_file = self.get_parameter('markers_output_file').value
        self.output_file = configured_output_file if configured_output_file else default_output_file
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        self.marker_lock = threading.Lock()
        self.saved_markers = []  # keep ALL detections

        self.patrol_running = False
        self.patrol_thread = None
        self.last_clusters = []
        self.last_face_goals = []

        self.robot_commander = RobotCommander(node_name='patrol_robot_commander')

        self.create_subscription(Marker, '/new_people_marker', self._marker_callback, 50)
        self.create_service(Trigger, '/start_patrol', self._start_patrol_callback)
        self.face_goals_pub = self.create_publisher(PoseArray, '/face_goals', 10)
        self.face_goals_markers_pub = self.create_publisher(MarkerArray, '/face_goals_markers', 10)

       #self.get_logger().info('PatrolPeopleCollector initialized. Service: /start_patrol')
        #self.get_logger().info(f'Markers output file: {self.output_file}')

    def _start_patrol_callback(self, request, response):
        del request
        if self.patrol_running:
            response.success = False
            response.message = 'Patrol already running.'
            return response

        self.patrol_running = True
        self.saved_markers = []
        self.last_clusters = []
        self.last_face_goals = []
        self._persist_markers()

        self.patrol_thread = threading.Thread(target=self._run_patrol, daemon=True)
        self.patrol_thread.start()

        response.success = True
        response.message = 'Patrol started.'
        return response

    def _run_patrol(self):
        try:
            #self.get_logger().info('Waiting for Nav2 to become active...')
            self.robot_commander.waitUntilNav2Active()

            # Undock if supported by this RobotCommander implementation
            if hasattr(self.robot_commander, 'undock'):
                try:
                    self.robot_commander.undock()
                    while not self.robot_commander.isTaskComplete():
                        rclpy.spin_once(self.robot_commander, timeout_sec=0.1)
                except Exception as e:
                   # self.get_logger().warn(f'Undock skipped/failed: {e}')
                   pass


            # Visit predefined patrol goals in fixed order
            for i, p in enumerate(self.predefined_positions, start=1):
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = p['position']['x']
                goal.pose.position.y = p['position']['y']
                goal.pose.position.z = p['position']['z']
                goal.pose.orientation.x = p['orientation']['x']
                goal.pose.orientation.y = p['orientation']['y']
                goal.pose.orientation.z = p['orientation']['z']
                goal.pose.orientation.w = p['orientation']['w']

                #self.get_logger().info(f'Patrol goal [{i}/{len(self.predefined_positions)}]')
                self.robot_commander.goToPose(goal)

                while not self.robot_commander.isTaskComplete():
                    rclpy.spin_once(self.robot_commander, timeout_sec=0.1)

                result = self.robot_commander.getResult()
                if result != TaskResult.SUCCEEDED:
                    #self.get_logger().warn(f'Patrol goal {i} failed with result: {result}')
                    pass
            # Cluster and create face goals
            print("kmal bo un line")
            clusters = self._cluster_markers()
            self.last_clusters = clusters
            face_goals = self._build_face_goals_from_clusters(clusters)
            self.last_face_goals = face_goals

            self._publish_face_goals(face_goals)
            self._publish_face_goal_markers(face_goals, clusters)

            # Visit face goals, greet. Retry mirrored side if failed.
            for i, (fg, c) in enumerate(zip(face_goals, clusters), start=1):
                #self.get_logger().info(f'Face goal [{i}/{len(face_goals)}] primary')
                self.robot_commander.goToPose(fg)
                while not self.robot_commander.isTaskComplete():
                    rclpy.spin_once(self.robot_commander, timeout_sec=0.1)

                result = self.robot_commander.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('hello')
                    continue

                #self.get_logger().warn(f'Primary face goal {i} failed, trying mirrored side...')
                alt = self._build_single_face_goal(c, sign=-1.0)
                self.robot_commander.goToPose(alt)
                while not self.robot_commander.isTaskComplete():
                    rclpy.spin_once(self.robot_commander, timeout_sec=0.1)

                result2 = self.robot_commander.getResult()
                if result2 == TaskResult.SUCCEEDED:
                    print('hello')
                else:
                    #self.get_logger().warn(f'Mirrored face goal {i} also failed: {result2}')
                    pass

            #self.get_logger().info('Patrol + face interaction completed.')

        except Exception as e:
            #self.get_logger().error(f'Patrol failed with exception: {e}')
            pass
        finally:
            try:
                self._persist_markers()
            except Exception:
                pass
            self.patrol_running = False

    def _marker_callback(self, marker_msg: Marker):
        # Prefer Marker.points encoding: points[0]=face position, points[1]=face+normal
        face_x = marker_msg.pose.position.x
        face_y = marker_msg.pose.position.y
        face_z = marker_msg.pose.position.z
        nx, ny, nz = 0.0, 0.0, 0.0

        if len(marker_msg.points) >= 2:
            p0 = marker_msg.points[0]
            p1 = marker_msg.points[1]
            face_x, face_y, face_z = p0.x, p0.y, p0.z
            nx, ny, nz = (p1.x - p0.x), (p1.y - p0.y), (p1.z - p0.z)

        vals = [face_x, face_y, face_z, nx, ny, nz]
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            return

        rec = {
            'stamp': self.get_clock().now().to_msg().sec,
            'frame_id': marker_msg.header.frame_id if marker_msg.header.frame_id else 'map',
            'marker_id': int(marker_msg.id),
            'position': {'x': float(face_x), 'y': float(face_y), 'z': float(face_z)},
            'normal': {'x': float(nx), 'y': float(ny), 'z': float(nz)},
        }

        with self.marker_lock:
            self.saved_markers.append(rec)

    def _cluster_markers(self):
        with self.marker_lock:
            points = deepcopy(self.saved_markers)

        r = self.cluster_radius_m
        clusters = []

        for m in points:
            px = m['position']['x']
            py = m['position']['y']
            assigned = False
            for c in clusters:
                cx = c['center']['x']
                cy = c['center']['y']
                if math.hypot(px - cx, py - cy) <= r:
                    c['markers'].append(m)
                    c['center'] = self._compute_cluster_center(c['markers'])
                    assigned = True
                    break

            if not assigned:
                clusters.append({
                    'markers': [m],
                    'center': {'x': px, 'y': py, 'z': m['position']['z']},
                })

        clusters.sort(key=lambda c: len(c['markers']), reverse=True)
        top3 = clusters[:3]

        # enrich with averaged normal
        for i, c in enumerate(top3, start=1):
            print("kva se dogaja")
            n = self._compute_cluster_normal(c['markers'])
            c['cluster_id'] = i
            c['size'] = len(c['markers'])
            c['normal'] = n

        return top3

    def _compute_cluster_center(self, cluster_markers):
        n = len(cluster_markers)
        sx = sum(m['position']['x'] for m in cluster_markers)
        sy = sum(m['position']['y'] for m in cluster_markers)
        sz = sum(m['position']['z'] for m in cluster_markers)
        return {'x': sx / n, 'y': sy / n, 'z': sz / n}

    def _compute_cluster_normal(self, cluster_markers):
        # Align normals first, then average (prevents cancellation/inversion)
        ref = None
        vecs = []

        for m in cluster_markers:
            #sou tele so 0 in zato ne dela
            nx = float(m['normal']['x'])
            ny = float(m['normal']['y'])
            nz = float(m['normal']['z'])
            n = nx * nx + ny * ny + nz * nz
            print(m['normal'])
            if n < 1e-6:
                continue
            nx, ny, nz = nx / n, ny / n, nz / n
            if ref is None:
                ref = (nx, ny, nz)
            else:
                dot = nx * ref[0] + ny * ref[1] + nz * ref[2]
                if dot < 0.0:
                    nx, ny, nz = -nx, -ny, -nz
            vecs.append((nx, ny, nz))
            print("kva se dogaja2")
            self.get_logger().info(f'Normal vector: ({nx:.2f}, {ny:.2f}, {nz:.2f})')

        if not vecs:
            return {'x': 1.0, 'y': 0.0, 'z': 0.0}

        sx = sum(v[0] for v in vecs)
        sy = sum(v[1] for v in vecs)
        sz = sum(v[2] for v in vecs)
        s = math.sqrt(sx * sx + sy * sy + sz * sz)
        if s < 1e-6:
            return {'x': 1.0, 'y': 0.0, 'z': 0.0}
        return {'x': sx / s, 'y': sy / s, 'z': sz / s}

    def _build_single_face_goal(self, c, sign=1.0):
        cx, cy = c['center']['x'], c['center']['y']
        nx, ny = c['normal']['x'], c['normal']['y']
        nxy = math.hypot(nx, ny)
        if nxy < 1e-6:
            nx, ny, nxy = 1.0, 0.0, 1.0
        nx /= nxy
        ny /= nxy

        gx = cx + sign * self.face_goal_offset_m * nx
        gy = cy + sign * self.face_goal_offset_m * ny

        # always face the face center
        yaw = math.atan2(cy - gy, cx - gx)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _build_face_goals_from_clusters(self, clusters):
        return [self._build_single_face_goal(c, sign=1.0) for c in clusters]

    def _publish_face_goals(self, goals):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = []

        for g in goals:
            p = Pose()
            p.position = g.pose.position
            p.orientation = g.pose.orientation
            msg.poses.append(p)

        self.face_goals_pub.publish(msg)
#        self.get_logger().info(f'Published {len(msg.poses)} face goals to /face_goals')

    def _publish_face_goal_markers(self, goals, clusters):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (g, c) in enumerate(zip(goals, clusters)):
            # Goal sphere
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = now
            goal_marker.ns = 'face_goals'
            goal_marker.id = i * 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose = g.pose
            goal_marker.scale.x = 0.18
            goal_marker.scale.y = 0.18
            goal_marker.scale.z = 0.18
            goal_marker.color.a = 1.0
            goal_marker.color.r = 0.1
            goal_marker.color.g = 0.9
            goal_marker.color.b = 0.1
            marker_array.markers.append(goal_marker)

            # Arrow from goal -> face center (shows facing/interact direction)
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'face_goal_arrows'
            arrow.id = i * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.04   # shaft diameter
            arrow.scale.y = 0.08   # head diameter
            arrow.scale.z = 0.10   # head length
            arrow.color.a = 1.0
            arrow.color.r = 1.0
            arrow.color.g = 0.3
            arrow.color.b = 0.1

            p_goal = deepcopy(g.pose.position)
            p_face = Pose().position
            p_face.x = float(c['center']['x'])
            p_face.y = float(c['center']['y'])
            p_face.z = float(c['center']['z'])
            arrow.points = [p_face, p_goal]
            marker_array.markers.append(arrow)

        self.face_goals_markers_pub.publish(marker_array)
#        self.get_logger().info(f'Published {len(marker_array.markers)} markers to /face_goals_markers')

    def _persist_markers(self):
        with self.marker_lock:
            raw = deepcopy(self.saved_markers)

        data = {
            'cluster_radius_m': self.cluster_radius_m,
            'face_goal_offset_m': self.face_goal_offset_m,
            'total_detections': len(raw),
            'top_3_clusters': [
                {
                    'cluster_id': c['cluster_id'],
                    'size': c['size'],
                    'center': c['center'],
                    'normal': c['normal'],
                }
                for c in self.last_clusters
            ],
            'face_goals': [
                {
                    'x': g.pose.position.x,
                    'y': g.pose.position.y,
                    'z': g.pose.position.z,
                    'qx': g.pose.orientation.x,
                    'qy': g.pose.orientation.y,
                    'qz': g.pose.orientation.z,
                    'qw': g.pose.orientation.w,
                }
                for g in self.last_face_goals
            ],
            'detections': raw,
        }

        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Saved results to {self.output_file}')

    def destroy_node(self):
        try:
            self.robot_commander.destroy_node()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolPeopleCollector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
