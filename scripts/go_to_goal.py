#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Envia un goal a Nav2, verifica aceptación del Action, graba rosbag con rosbag2_py.
Con FREEZE_START_POSE=True NO se actualiza start_pose.json (se usa el cache fijo).
NO publica initialpose por defecto (no pisa tu 2D Pose Estimate).
Opcional: --publish-initialpose para publicarla si la necesitas.

Uso:
  python3 go_to_goal.py --controller mppi --trial 1 --goal 3.5 2.0 0.0
  # opcional:
  python3 go_to_goal.py --controller mppi --trial 1 --publish-initialpose
"""

import os, json, time, math, argparse, sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

# --------- CONFIG ----------
NAMESPACE = "/a200_0000"
ODOM_TOPIC = f"{NAMESPACE}/platform/odom/filtered"
AMCL_POSE_TOPIC = f"{NAMESPACE}/amcl_pose"
INITPOSE_TOPIC = f"{NAMESPACE}/initialpose"
GOAL_MARKER_TOPIC = f"{NAMESPACE}/goal_pose"
CMD_TOPIC = f"{NAMESPACE}/cmd_vel"
PLAN_TOPIC = f"{NAMESPACE}/plan"
RECEIVED_GLOBAL_PLAN_TOPIC = f"{NAMESPACE}/received_global_plan"
CLOCK_TOPIC = "/clock"

# TÓPICOS EXTRA QUE ESTÁS GRABANDO
LOCAL_PLAN_TOPIC = f"{NAMESPACE}/local_plan"
LOCAL_COSTMAP_TOPIC = f"{NAMESPACE}/local_costmap/costmap"
LIDAR2D_SCAN_TOPIC = f"{NAMESPACE}/sensors/lidar2d_0/scan"
DIAG_NS_TOPIC = f"{NAMESPACE}/diagnostics"
DIAG_GLOBAL_TOPIC = "/diagnostics"

DEFAULT_GOAL = [3.5, 2.0, 0.0]   # x, y, yaw(rad)
EXTRA_RECORD_SECONDS = 5.0        # s tras recibir resultado
ACTION_TIMEOUT_SEC = 400.0        # s timeout total
AMCL_WAIT_SEC = 10.0              # (se mantiene por compatibilidad, pero no se usa si FREEZE_START_POSE=True)

CACHE_DIR = os.path.expanduser("~/ros2_ws/src/kevin_nav2_experiments/results/_cache")
os.makedirs(CACHE_DIR, exist_ok=True)
START_POSE_JSON = os.path.join(CACHE_DIR, "start_pose.json")

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# CONGELAR CACHE (no escribir start_pose.json durante los trials)
FREEZE_START_POSE = True
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

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
            (AMCL_POSE_TOPIC, 'geometry_msgs/msg/PoseWithCovarianceStamped'),
            (GOAL_MARKER_TOPIC, 'geometry_msgs/msg/PoseStamped'),
            # --- EXTRA ---
            (LOCAL_PLAN_TOPIC, 'nav_msgs/msg/Path'),
            (LOCAL_COSTMAP_TOPIC, 'nav_msgs/msg/OccupancyGrid'),
            (LIDAR2D_SCAN_TOPIC, 'sensor_msgs/msg/LaserScan'),
            (DIAG_NS_TOPIC, 'diagnostic_msgs/msg/DiagnosticArray'),
            (DIAG_GLOBAL_TOPIC, 'diagnostic_msgs/msg/DiagnosticArray')
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
            node.create_subscription(PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._cb(AMCL_POSE_TOPIC), qos),
            node.create_subscription(PoseStamped, GOAL_MARKER_TOPIC, self._cb(GOAL_MARKER_TOPIC), qos),
            # --- EXTRA ---
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

class GoToGoalNode(Node):
    def __init__(self, controller: str, trial: int, results_dir: str, goal_xyz, publish_initialpose: bool):
        super().__init__('go_to_goal_node')
        self.controller = controller
        self.trial = trial
        self.goal_xyz = goal_xyz
        self.publish_initialpose = publish_initialpose

        # Estado
        self._start_saved = FREEZE_START_POSE   # <- si True, NO intentará escribir start_pose.json
        self._last_amcl = None                  # mantenido por compatibilidad
        self._current_xy = None
        self._goal_sent = False
        self._goal_handle = None
        self._result_future = None
        self._start_time = self.get_clock().now()
        self._amcl_wait_deadline = self._start_time + Duration(seconds=AMCL_WAIT_SEC)

        # Pubs/subs
        qos = QoSProfile(depth=50)
        self._goal_marker_pub = self.create_publisher(PoseStamped, GOAL_MARKER_TOPIC, 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, INITPOSE_TOPIC, 10)

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._amcl_cb, 10)

        # Action client
        self._nav_client = ActionClient(self, NavigateToPose, f"{NAMESPACE}/navigate_to_pose")

        # Bag
        os.makedirs(os.path.join(results_dir, controller), exist_ok=True)
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._bag_uri = os.path.join(results_dir, controller, f"{controller}_forward_{trial:02d}_{now}")
        self.get_logger().info(f"[INFO] Bag: {self._bag_uri}")
        self._bag = RosbagRecorder(self, self._bag_uri)

        # Timer
        self.create_timer(0.1, self._step)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        # Mantener última AMCL por si algún día decides descongelar
        yaw = self._quat_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self._last_amcl = {
            "frame_id": msg.header.frame_id or "map",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw,
            "stamp": (msg.header.stamp.sec, msg.header.stamp.nanosec)
        }

    def _odom_cb(self, msg: Odometry):
        self._current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.publish_initialpose and not self._start_saved:
            yaw = self._quat_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            p = PoseWithCovarianceStamped()
            p.header.frame_id = "map"
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.pose.position.x = msg.pose.pose.position.x
            p.pose.pose.position.y = msg.pose.pose.position.y
            p.pose.pose.orientation.z = math.sin(yaw/2.0)
            p.pose.pose.orientation.w = math.cos(yaw/2.0)
            cov = [0.0]*36
            cov[0] = 0.25; cov[7] = 0.25; cov[35] = (10.0*math.pi/180.0)**2
            for _ in range(5):
                self._initialpose_pub.publish(p)
                time.sleep(0.05)
            self.get_logger().info("[INFO] initialpose PUBLICADA por petición (--publish-initialpose).")

    def _step(self):
        if not self._nav_client.server_is_ready():
            return

        # Si el cache está congelado, saltamos la lógica de escritura
        # y seguimos directo a enviar el goal cuando tengamos odom.
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

            self.get_logger().info(f"[INFO] Enviando goal -> ({gx:.2f}, {gy:.2f}, {gyaw:.2f} rad)")
            fut = self._nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
            fut.add_done_callback(self._goal_response_cb)

            self._goal_sent = True
            return

        if (self.get_clock().now() - self._start_time) > Duration(seconds=ACTION_TIMEOUT_SEC):
            self.get_logger().error("[ERROR] Timeout del experimento GO. Saliendo.")
            rclpy.shutdown()
            sys.exit(1)

    def _goal_response_cb(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[ERROR] Al enviar goal: {e}")
            rclpy.shutdown(); sys.exit(1); return

        if not self._goal_handle.accepted:
            self.get_logger().error("[ERROR] Goal RECHAZADO por bt_navigator.")
            rclpy.shutdown(); sys.exit(2); return

        self.get_logger().info("[OK] Goal ACEPTADO.")
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
            self.get_logger().warn(f"[WARN] Sin resultado GO: {e}")

        self.get_logger().info(f"[INFO] Esperando {EXTRA_RECORD_SECONDS}s y cerrando bag...")
        time.sleep(EXTRA_RECORD_SECONDS)
        rclpy.shutdown()

    @staticmethod
    def _quat_to_yaw(x,y,z,w):
        siny_cosp = 2.0*(w*z + x*y); cosy_cosp = 1.0 - 2.0*(y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True, help="Etiqueta del controlador para nombrar el bag.")
    parser.add_argument("--trial", type=int, default=1)
    parser.add_argument("--results-dir", default=os.path.expanduser("~/ros2_ws/src/kevin_nav2_experiments/results"))
    parser.add_argument("--goal", type=float, nargs=3, default=DEFAULT_GOAL, help="x y yaw(rad)")
    parser.add_argument("--publish-initialpose", action="store_true", help="(opcional) Publicar initialpose desde odom si la necesitas")
    args = parser.parse_args()

    rclpy.init()
    node = GoToGoalNode(controller=args.controller, trial=args.trial,
                        results_dir=args.results_dir, goal_xyz=args.goal,
                        publish_initialpose=args.publish_initialpose)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

