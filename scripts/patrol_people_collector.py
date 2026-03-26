#!/usr/bin/env python3

import json
import math
import os
import threading
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from rclpy.executors import MultiThreadedExecutor


from robot_commander import RobotCommander, TaskResult


class PatrolPeopleCollector(Node):
    def __init__(self):
        super().__init__('patrol_people_collector')

        self.declare_parameter('dedup_distance_m', 0.25)
        self.declare_parameter('markers_output_file', '')

        self.dedup_distance_m = float(self.get_parameter('dedup_distance_m').value)
        self.predefined_positions = [
            {
                'position': {'x': -0.18800890986934185, 'y': 2.2445737881641175, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9559878810605652, 'w': 0.29340615410268844},
            },
            {
                'position': {'x': -0.6260109818580369, 'y': 4.090310414909066, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9996510645649817, 'w': 0.02641494111481136},
            },
            {
                'position': {'x': -2.6823971577911045, 'y': 3.674855375367437, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9549465972545198, 'w': 0.2967776885010294},
            },
            {
                'position': {'x': -2.785116767203785, 'y': 1.8051517502252235, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.5436931322523505, 'w': 0.8392840865533124},
            },
            {
                'position': {'x': -2.1631123925949387, 'y': 0.16004909911389464, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.08786734960939964, 'w': 0.9961321844376978},
            },
            {
                'position': {'x': 0.4364746411129452, 'y': 0.10124296846345447, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.6835235458166421, 'w': 0.7299284638334395},
            },
            {
                'position': {'x': -0.8007714613378948, 'y': 3.4270642313164883, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9999088947569018, 'w': 0.013498228996092689},
            },
        ]

        # Determine the workspace root by looking for colcon_ws
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parts = script_dir.split(os.sep)
        if 'colcon_ws' in parts:
            workspace_idx = parts.index('colcon_ws')
            workspace_root = os.sep + os.path.join(*parts[:workspace_idx + 1])
        else:
            workspace_root = os.path.expanduser('~')
        
        default_output_file = os.path.join(
            workspace_root,
            'src',
            'dis_tutorial3',
            'data',
            'detected_people.json',
        )
        configured_output_file = self.get_parameter('markers_output_file').value
        self.output_file = configured_output_file if configured_output_file else default_output_file

        self.marker_lock = threading.Lock()
        self.saved_markers = []

        self.patrol_running = False
        self.patrol_thread = None

        self.create_subscription(Marker, '/new_people_marker', self._marker_callback, 10)
        self.create_service(Trigger, '/start_patrol', self._start_patrol_callback)

        self.robot_commander = RobotCommander(node_name='patrol_robot_commander')

        self.get_logger().info('PatrolPeopleCollector initialized. Service: /start_patrol')
        self.get_logger().info(f'Markers output file: {self.output_file}')

    def _start_patrol_callback(self, request, response):
        del request

        if self.patrol_running:
            response.success = False
            response.message = 'Patrol already running.'
            return response

        self.patrol_running = True
        self.patrol_thread = threading.Thread(target=self._run_patrol, daemon=True)
        self.patrol_thread.start()

        response.success = True
        response.message = 'Patrol started.'
        return response

    def _run_patrol(self):
        try:
            self.get_logger().info('Waiting for Nav2 to become active...')
            self.robot_commander.waitUntilNav2Active()
            self.get_logger().info('Nav2 is active. Checking dock status...')

            # Wait until dock_status is received
            max_dock_wait = 10
            dock_wait_count = 0
            while self.robot_commander.is_docked is None and rclpy.ok() and dock_wait_count < max_dock_wait:
                rclpy.spin_once(self.robot_commander, timeout_sec=0.2)
                dock_wait_count += 1

            if self.robot_commander.is_docked is None:
                self.get_logger().warn('Dock status never received, assuming not docked and continuing.')
            elif self.robot_commander.is_docked:
                self.get_logger().info('Robot is docked, undocking first...')
                self.robot_commander.undock()
                self.get_logger().info('Undocking complete.')
            else:
                self.get_logger().info('Robot is already undocked.')

            self.get_logger().info(f'Starting patrol of {len(self.predefined_positions)} waypoints...')

            for index, target_pose in enumerate(self.predefined_positions, start=1):
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.robot_commander.get_clock().now().to_msg()
                goal_pose.pose.position.x = float(target_pose['position']['x'])
                goal_pose.pose.position.y = float(target_pose['position']['y'])
                goal_pose.pose.position.z = float(target_pose['position']['z'])
                goal_pose.pose.orientation.x = float(target_pose['orientation']['x'])
                goal_pose.pose.orientation.y = float(target_pose['orientation']['y'])
                goal_pose.pose.orientation.z = float(target_pose['orientation']['z'])
                goal_pose.pose.orientation.w = float(target_pose['orientation']['w'])

                self.get_logger().info(
                    f'[{index}/{len(self.predefined_positions)}] Sending goal to: '
                    f'x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}, '
                    f'z={goal_pose.pose.position.z:.3f}'
                )

                # Reset goal state before sending new goal
                self.robot_commander.goal_handle = None
                self.robot_commander.result_future = None
                self.robot_commander.status = None

                accepted = self.robot_commander.goToPose(goal_pose)
                if not accepted:
                    self.get_logger().error(f'Goal {index} was rejected by Nav2, skipping to next waypoint.')
                    continue

                self.get_logger().info(f'Goal {index} accepted by Nav2, waiting for completion...')
                
                # Wait for task completion with periodic logging
                loop_count = 0
                while not self.robot_commander.isTaskComplete() and rclpy.ok():
                    loop_count += 1
                    if loop_count % 20 == 0:  # Log every 10 seconds (20 * 0.5)
                        self.get_logger().info(f'Goal {index} still in progress...')
                    time.sleep(0.5)

                result = self.robot_commander.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'✓ Waypoint {index} reached successfully.')
                else:
                    self.get_logger().warn(f'⚠ Waypoint {index} finished with result: {result.name}')

            self.get_logger().info('All waypoints visited. Persisting and clustering marker data...')
            self._persist_markers()
            self.get_logger().info(f'Patrol complete! Processed {len(self.saved_markers)} total detections.')
        except Exception as exc:
            self.get_logger().error(f'Patrol failed with exception: {exc}', exc_info=True)
        finally:
            self.patrol_running = False

    def _marker_callback(self, marker_msg: Marker):
        # Extract and validate marker position
        x = float(marker_msg.pose.position.x)
        y = float(marker_msg.pose.position.y)
        z = float(marker_msg.pose.position.z)

        # Skip markers with invalid (NaN) positions
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            self.get_logger().debug(
                f'Skipping marker with NaN position: id={marker_msg.id}, '
                f'x={x}, y={y}, z={z}'
            )
            return

        marker_record = {
            'id': int(marker_msg.id),
            'frame_id': marker_msg.header.frame_id,
            'stamp_sec': int(marker_msg.header.stamp.sec),
            'stamp_nanosec': int(marker_msg.header.stamp.nanosec),
            'x': x,
            'y': y,
            'z': z,
        }

        with self.marker_lock:
            self.saved_markers.append(marker_record)

        self.get_logger().debug(
            f'Saved marker at ({marker_record["x"]:.2f}, {marker_record["y"]:.2f}, {marker_record["z"]:.2f}). '
            f'Total collected: {len(self.saved_markers)}'
        )

    def _cluster_markers(self):
        """
        Cluster markers using greedy distance-based clustering.
        Returns list of clusters, each cluster is a list of marker records.
        """
        if not self.saved_markers:
            return []

        clustering_threshold = self.dedup_distance_m
        clusters = []
        used_indices = set()

        for i, marker in enumerate(self.saved_markers):
            if i in used_indices:
                continue

            cluster = [marker]
            used_indices.add(i)

            for j, other_marker in enumerate(self.saved_markers):
                if j in used_indices:
                    continue

                distance = math.dist(
                    (marker['x'], marker['y'], marker['z']),
                    (other_marker['x'], other_marker['y'], other_marker['z']),
                )

                if distance <= clustering_threshold:
                    cluster.append(other_marker)
                    used_indices.add(j)

            clusters.append(cluster)

        return clusters

    def _persist_markers(self):
        output_dir = os.path.dirname(self.output_file)
        os.makedirs(output_dir, exist_ok=True)

        with self.marker_lock:
            all_markers = deepcopy(self.saved_markers)

        # Cluster all collected markers
        self.get_logger().info(f'Clustering {len(all_markers)} collected markers...')
        clusters = self._cluster_markers()
        self.get_logger().info(f'Found {len(clusters)} clusters')

        # Sort clusters by size (descending) and keep top 3
        sorted_clusters = sorted(clusters, key=len, reverse=True)
        top_clusters = sorted_clusters[:3]

        self.get_logger().info(f'Keeping top 3 clusters with sizes: {[len(c) for c in top_clusters]}')

        # Prepare payload with clustering info
        clusters_data = []
        for cluster_idx, cluster in enumerate(top_clusters, start=1):
            cluster_center = self._compute_cluster_center(cluster)
            clusters_data.append({
                'cluster_id': cluster_idx,
                'size': len(cluster),
                'center': cluster_center,
                'markers': cluster,
            })

        payload = {
            'clustering_threshold_m': self.dedup_distance_m,
            'total_detections': len(all_markers),
            'total_clusters': len(clusters),
            'top_3_clusters': clusters_data,
        }

        with open(self.output_file, 'w', encoding='utf-8') as output_handle:
            json.dump(payload, output_handle, indent=2)

    def _compute_cluster_center(self, cluster):
        """
        Compute the centroid of a cluster of markers.
        """
        if not cluster:
            return {'x': 0, 'y': 0, 'z': 0}

        avg_x = sum(m['x'] for m in cluster) / len(cluster)
        avg_y = sum(m['y'] for m in cluster) / len(cluster)
        avg_z = sum(m['z'] for m in cluster) / len(cluster)

        return {'x': avg_x, 'y': avg_y, 'z': avg_z}

    def destroy_node(self):
        try:
            self._persist_markers()
        except Exception as exc:
            self.get_logger().warn(f'Could not persist markers on shutdown: {exc}')
        self.robot_commander.destroyNode()
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
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
