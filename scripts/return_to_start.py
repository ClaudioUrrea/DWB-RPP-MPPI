#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, time, math, argparse, sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist   # <-- Twist agregado
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

NAMESPACE = "/a200_0000"
ODOM_TOPIC = f"{NAMESPACE}/platform/odom/filtered"
GOAL_MARKER_TOPIC = f"{NAMESPACE}/goal_pose"
CMD_TOPIC = f"{NAMESPACE}/cmd_vel"
PLAN_TOPIC = f"{NAMESPACE}/plan"
RECEIVED_GLOBAL_PLAN_TOPIC = f"{NAMESPACE}/received_global_plan"
CLOCK_TOPIC = "/clock"

# NUEVOS TÓPICOS A GRABAR (consistencia con GO)
AMCL_POSE_TOPIC = f"{NAMESPACE}/amcl_pose"
LOCAL_PLAN_TOPIC = f"{NAMESPACE}/local_plan"
LOCAL_COSTMAP_TOPIC = f"{NAMESPACE}/local_costmap/costmap"
LIDAR2D_SCAN_TOPIC = f"{NAMESPACE}/sensors/lidar2d_0/scan"
DIAG_NS_TOPIC = f"{NAMESPACE}/diagnostics"
DIAG_GLOBAL_TOPIC = "/diagnostics"

DEFAULT_RETURN = [0.0, 0.0, 0.0]
ARRIVAL_DIST_THRESH = 0.15
EXTRA_RECORD_SECONDS = 5.0
ACTION_TIMEOUT_SEC = 300.0

CACHE_DIR = os.path.expanduser("~/ros2_ws/src/kevin_nav2_experiments/results/_cache")
os.makedirs(CACHE_DIR, exist_ok=True)
START_POSE_JSON = os.path.join(CACHE_DIR, "start_pose.json")

def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class RosbagRecorder:
    def __init__(self, node: Node, bag_uri: str):
        self._node = node
        self._clk = node.get_clock()
        self._writer = SequentialWriter()
        self._writer.open(StorageOptions(uri=bag_uri, storage_id='sqlite3'), ConverterOptions('', ''))
        topics = [
            (ODOM_TOPIC, 'nav_msgs/msg/Odometry'),
            (CMD_TOPIC, 'geometry_msgs/msg/Twist'),
            (PLAN_TOPIC, 'nav_msgs/msg/Path'),
            (RECEIVED_GLOBAL_PLAN_TOPIC, 'nav_msgs/msg/Path'),
            (CLOCK_TOPIC, 'rosgraph_msgs/msg/Clock'),
            (GOAL_MARKER_TOPIC, 'geometry_msgs/msg/PoseStamped'),
            # --- NUEVOS ---
            (AMCL_POSE_TOPIC, 'geometry_msgs/msg/PoseWithCovarianceStamped'),
            (LOCAL_PLAN_TOPIC, 'nav_msgs/msg/Path'),
            (LOCAL_COSTMAP_TOPIC, 'nav_msgs/msg/OccupancyGrid'),
            (LIDAR2D_SCAN_TOPIC, 'sensor_msgs/msg/LaserScan'),
            (DIAG_NS_TOPIC, 'diagnostic_msgs/msg/DiagnosticArray'),
            (DIAG_GLOBAL_TOPIC, 'diagnostic_msgs/msg/DiagnosticArray'),
        ]
        for name, typ in topics:
            self._writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format='cdr'))
        qos = QoSProfile(depth=50)
        self._subs = [
            node.create_subscription(Odometry, ODOM_TOPIC, self._cb(ODOM_TOPIC), qos),
            node.create_subscription(Twist, CMD_TOPIC, self._cb(CMD_TOPIC), qos),
            node.create_subscription(Path, PLAN_TOPIC, self._cb(PLAN_TOPIC), qos),
            node.create_subscription(Path, RECEIVED_GLOBAL_PLAN_TOPIC, self._cb(RECEIVED_GLOBAL_PLAN_TOPIC), qos),
            node.create_subscription(Clock, CLOCK_TOPIC, self._cb(CLOCK_TOPIC), qos),
            node.create_subscription(PoseStamped, GOAL_MARKER_TOPIC, self._cb(GOAL_MARKER_TOPIC), qos),
            # --- NUEVAS SUSCRIPCIONES ---
            node.create_subscription(PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._cb(AMCL_POSE_TOPIC), qos),
            node.create_subscription(Path, LOCAL_PLAN_TOPIC, self._cb(LOCAL_PLAN_TOPIC), qos),
            node.create_subscription(OccupancyGrid, LOCAL_COSTMAP_TOPIC, self._cb(LOCAL_COSTMAP_TOPIC), qos),
            node.create_subscription(LaserScan, LIDAR2D_SCAN_TOPIC, self._cb(LIDAR2D_SCAN_TOPIC), qos),
            node.create_subscription(DiagnosticArray, DIAG_NS_TOPIC, self._cb(DIAG_NS_TOPIC), qos),
            node.create_subscription(DiagnosticArray, DIAG_GLOBAL_TOPIC, self._cb(DIAG_GLOBAL_TOPIC), qos),
        ]
    def _cb(self, topic_name):
        def writer(msg):
            try:
                self._writer.write(topic_name, serialize_message(msg), self._clk.now().nanoseconds)
            except Exception as e:
                self._node.get_logger().warn(f"[rosbag] Error escribiendo {topic_name}: {e}")
        return writer

class ReturnToStartNode(Node):
    def __init__(self, controller: str, trial: int, results_dir: str, fallback_xyz):
        super().__init__('return_to_start_node')

        # Cargar start pose (en frame map) desde cache
        if os.path.exists(START_POSE_JSON):
            with open(START_POSE_JSON, "r") as f:
                d = json.load(f)
            self.goal_xyz = [float(d.get("x", 0.0)), float(d.get("y", 0.0)), float(d.get("yaw", 0.0))]
            self.get_logger().info(f"[INFO] START pose (MAP) desde cache: {self.goal_xyz}")
        else:
            self.goal_xyz = fallback_xyz
            self.get_logger().warn(f"[WARN] No hay cache de start. Usando fallback (map): {self.goal_xyz}")

        self._current_xy = None
        self._goal_sent = False
        self._goal_handle = None
        self._result_future = None
        self._start_time = self.get_clock().now()

        qos = QoSProfile(depth=50)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, qos)
        self._goal_marker_pub = self.create_publisher(PoseStamped, GOAL_MARKER_TOPIC, 10)

        self._nav_client = ActionClient(self, NavigateToPose, f"{NAMESPACE}/navigate_to_pose")

        os.makedirs(os.path.join(results_dir, controller), exist_ok=True)
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._bag_uri = os.path.join(results_dir, controller, f"{controller}_return_{trial:02d}_{now}")
        self.get_logger().info(f"[INFO] Bag: {self._bag_uri}")
        self._bag = RosbagRecorder(self, self._bag_uri)

        self.create_timer(0.1, self._step)

    def _odom_cb(self, msg: Odometry):
        self._current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _step(self):
        if not self._nav_client.server_is_ready():
            return

        if not self._goal_sent and self._current_xy is not None:
            gx, gy, gyaw = self.goal_xyz
            qx,qy,qz,qw = yaw_to_quat(gyaw)

            viz = PoseStamped()
            viz.header.frame_id = "map"
            viz.header.stamp = self.get_clock().now().to_msg()
            viz.pose.position.x = gx
            viz.pose.position.y = gy
            viz.pose.orientation.x = qx
            viz.pose.orientation.y = qy
            viz.pose.orientation.z = qz
            viz.pose.orientation.w = qw
            self._goal_marker_pub.publish(viz)

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = viz.header.stamp
            goal.pose.pose = viz.pose

            self.get_logger().info(f"[INFO] Enviando RETORNO -> ({gx:.2f}, {gy:.2f}, {gyaw:.2f} rad) [frame=map]")
            fut = self._nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
            fut.add_done_callback(self._goal_response_cb)

            self._goal_sent = True
            return

        if (self.get_clock().now() - self._start_time) > Duration(seconds=ACTION_TIMEOUT_SEC):
            self.get_logger().error("[ERROR] Timeout retorno. Saliendo.")
            rclpy.shutdown(); sys.exit(1)

    def _goal_response_cb(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[ERROR] Al enviar RETORNO: {e}")
            rclpy.shutdown(); sys.exit(1); return

        if not self._goal_handle.accepted:
            self.get_logger().error("[ERROR] RETORNO RECHAZADO por bt_navigator.")
            rclpy.shutdown(); sys.exit(2); return

        self.get_logger().info("[OK] RETORNO ACEPTADO.")
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback):
        try:
            dr = getattr(feedback.feedback, "distance_remaining", None)
            if dr is not None:
                self.get_logger().info(f"[FB] dist_restante={dr:.2f} m")
        except Exception:
            pass

    def _result_cb(self, future):
        try:
            res = future.result()
            status = getattr(res, "status", None)
            self.get_logger().info(f"[RESULT] status={status}")
        except Exception as e:
            self.get_logger().warn(f"[WARN] Sin resultado retorno: {e}")

        self.get_logger().info(f"[INFO] Esperando {EXTRA_RECORD_SECONDS}s y cerrando bag...")
        time.sleep(EXTRA_RECORD_SECONDS)
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True, help="Etiqueta del controlador para nombre de bag.")
    parser.add_argument("--trial", type=int, default=1)
    parser.add_argument("--results-dir", default=os.path.expanduser("~/ros2_ws/src/kevin_nav2_experiments/results"))
    parser.add_argument("--fallback-return", type=float, nargs=3, default=DEFAULT_RETURN, help="x y yaw(rad) si no hay cache")
    args = parser.parse_args()

    rclpy.init()
    node = ReturnToStartNode(controller=args.controller, trial=args.trial,
                             results_dir=args.results_dir, fallback_xyz=args.fallback_return)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

