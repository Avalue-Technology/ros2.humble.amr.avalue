"""
/*
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Name: follow_waypoints_click.py
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Purpose: Listen rviz2 Waypoint until press ENTER to send whole waypoints to the navigation topic
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Dependent Reference
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	(01)Python 3.8.10
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	(02)Please run: Start Navigation -> RViz2-NAV2 -> Follow Waypoints Click
 *--------------------------------------------------------------------------------------------------------------------------------------> 
 *	Known Issues
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Methodology
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	References
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	MSDN documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Internal documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Internet documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
*/
"""

#!/usr/bin/env python3
import os
import sys
import math
import json
import threading
import termios
import tty
import select
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException

# =======================
# Config
# =======================
MAP_RESOLUTION = 0.05                 # meters per cell (for reference)
WAYPOINT_ARROW_LENGTH_M = 0.20        # arrow shaft length
NEAR_SKIP_THRESH_M = 0.20             # if robot is within this distance of first waypoint, skip it

LOOP_INTERVAL_MS = 5000               # per-lap delay after completion

SAVE_PATH = os.path.expanduser("~/.follow_waypoints_click.json")
TOPIC_PREVIEW = "/waypoint_preview"

CLICK_TOPIC = "/clicked_point"        # RViz2 Publish Point publishes here
DEFAULT_FRAME = "map"                 # fallback if incoming msg has empty frame_id
BASE_FRAME = "base_footprint"         # IMPORTANT: match your nav2 config

# KeySequences
KEY_ENTER_CR = "\r"
KEY_ENTER_LF = "\n"
KEY_CTRL_C   = "\x03"
KEY_F6       = "\x1b[17~"  # Toggle loop
KEY_F7       = "\x1b[18~"  # Load & Send (single or loop)
KEY_F8       = "\x1b[19~"  # Save

# =======================
# Math helpers
# =======================
def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

# =======================
# Raw key reader
# =======================
class RawKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def readkey(self, timeout=0.02) -> str:
        ch = os.read(self.fd, 1).decode("utf-8", errors="ignore")
        if ch != "\x1b":
            return ch
        seq = ch
        while True:
            r, _, _ = select.select([self.fd], [], [], timeout)
            if not r:
                break
            seq += os.read(self.fd, 1).decode("utf-8", errors="ignore")
            if seq.endswith("~"):
                break
            if len(seq) >= 3 and seq.startswith("\x1bO"):
                break
        return seq

# =======================
# WaypointCollector
# =======================
class WaypointCollector(Node):
    """Collect RViz2 Publish Point (/clicked_point), preview, and send to /follow_waypoints."""

    def __init__(self):
        super().__init__('waypoint_collector')

        # Loop state
        self.LOOP_FLAG = False
        self._loop_timer = None
        self._loop_poses_raw: List[PoseStamped] = []
        self._goal_active = False
        self._pingpong_reverse_next = False

        # Subscriptions / Clients
        self.sub = self.create_subscription(PointStamped, CLICK_TOPIC, self._OnClicked, 10)
        self.cli = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.vis_pub = self.create_publisher(MarkerArray, TOPIC_PREVIEW, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pose storage
        self._poses: List[PoseStamped] = []
        self._lock = threading.Lock()

        # Marker bookkeeping
        self._last_marker_count = 0

        # Request flags
        self._send_requested = False
        self._load_send_requested = False
        self._poll_timer = self.create_timer(0.1, self._PollRequests)

        # Banner
        self.get_logger().info("============================================================")
        self.get_logger().info(" RViz2: use 'Publish Point' tool to click waypoints on map")
        self.get_logger().info(f"   Topic : {CLICK_TOPIC}")
        self.get_logger().info(f"   Base  : {BASE_FRAME}")
        self.get_logger().info("   Enter : Send current waypoints")
        self.get_logger().info("     F6  : Toggle LOOP_FLAG (Ping-Pong Loop)")
        self.get_logger().info(f"     F7  : Load saved waypoints and send (Lap Delay {LOOP_INTERVAL_MS} ms)")
        self.get_logger().info("     F8  : Save current waypoints")
        self.get_logger().info("  Ctrl+C : Exit")
        self.get_logger().info("============================================================\n")

        self._PublishPreview()

    # RViz2 Publish Point callback
    def _OnClicked(self, msg: PointStamped):
        frame = msg.header.frame_id.strip() if msg.header.frame_id else DEFAULT_FRAME

        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = msg.header.stamp  # keep time if you want
        ps.pose.position.x = msg.point.x
        ps.pose.position.y = msg.point.y
        ps.pose.position.z = msg.point.z
        ps.pose.orientation.w = 1.0

        with self._lock:
            self._poses.append(ps)
            count = len(self._poses)

        self.get_logger().info(
            f"PublishPoint clicked: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) frame={frame}  (collected: {count})"
        )
        self._PublishPreview()

    # Robot pose helpers
    def _GetRobotPose(self, frame: str) -> Optional[tuple]:
        """Return (x, y, yaw) of BASE_FRAME in given frame, or None if TF unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(frame, BASE_FRAME, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return (t.x, t.y, yaw)
        except TransformException as e:
            self.get_logger().warn(f"TF unavailable for {frame}->{BASE_FRAME}: {e}")
            return None

    def _GetRobotYaw(self, frame: str) -> Optional[float]:
        p = self._GetRobotPose(frame)
        return p[2] if p is not None else None

    # Yaw computation
    def _ComputeYawsForSend(self, poses: List[PoseStamped]) -> List[float]:
        n = len(poses)
        if n == 0:
            return []
        frame = poses[0].header.frame_id or DEFAULT_FRAME

        if n == 1:
            yaw0 = self._GetRobotYaw(frame)
            return [yaw0 if yaw0 is not None else 0.0]

        yaws: List[float] = []
        robot_yaw = self._GetRobotYaw(frame)
        if robot_yaw is not None:
            yaws.append(robot_yaw)
        else:
            x0, y0 = poses[0].pose.position.x, poses[0].pose.position.y
            x1, y1 = poses[1].pose.position.x, poses[1].pose.position.y
            yaws.append(math.atan2(y1 - y0, x1 - x0))

        for i in range(1, n - 1):
            x0, y0 = poses[i].pose.position.x, poses[i].pose.position.y
            x1, y1 = poses[i + 1].pose.position.x, poses[i + 1].pose.position.y
            yaws.append(math.atan2(y1 - y0, x1 - x0))

        yaws.append(yaws[-1])
        return yaws

    @staticmethod
    def _ApplyYawsToPoses(poses: List[PoseStamped], yaws: List[float]) -> List[PoseStamped]:
        out: List[PoseStamped] = []
        for ps, yaw in zip(poses, yaws):
            nx = PoseStamped()
            nx.header = ps.header
            nx.pose.position = ps.pose.position
            qx, qy, qz, qw = yaw_to_quat(yaw)
            nx.pose.orientation.x = qx
            nx.pose.orientation.y = qy
            nx.pose.orientation.z = qz
            nx.pose.orientation.w = qw
            out.append(nx)
        return out

    @staticmethod
    def _CopyPoses(src: List[PoseStamped]) -> List[PoseStamped]:
        out: List[PoseStamped] = []
        for p in src:
            q = PoseStamped()
            q.header = p.header
            q.pose.position.x = float(p.pose.position.x)
            q.pose.position.y = float(p.pose.position.y)
            q.pose.position.z = float(getattr(p.pose.position, "z", 0.0))
            q.pose.orientation = p.pose.orientation
            out.append(q)
        return out

    @staticmethod
    def _ReversePoses(src: List[PoseStamped]) -> List[PoseStamped]:
        return list(reversed(WaypointCollector._CopyPoses(src)))

    def _BuildLapPoses(self, base: List[PoseStamped], forward: bool) -> List[PoseStamped]:
        route = self._CopyPoses(base) if forward else self._ReversePoses(base)
        if not route:
            return route

        frame = route[0].header.frame_id or DEFAULT_FRAME
        rp = self._GetRobotPose(frame)
        if rp is not None:
            rx, ry, _ = rp
            dx = route[0].pose.position.x - rx
            dy = route[0].pose.position.y - ry
            if math.hypot(dx, dy) < NEAR_SKIP_THRESH_M:
                route = route[1:]
                if not route:
                    return []
        return route

    # Preview
    def _PublishPreview(self):
        with self._lock:
            poses = list(self._poses)

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        if not poses:
            self._ClearPreview()
            return

        frame = poses[0].header.frame_id or DEFAULT_FRAME

        # Path line
        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = now
        line.ns = 'waypoints'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0
        line.points = [Point(x=p.pose.position.x, y=p.pose.position.y, z=0.0) for p in poses]
        ma.markers.append(line)

        # Arrows
        n = len(poses)
        for i in range(n):
            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.header.stamp = now
            arrow.ns = 'waypoints'
            arrow.id = 100 + i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = WAYPOINT_ARROW_LENGTH_M
            arrow.scale.y = 0.08
            arrow.scale.z = 0.12
            arrow.color.r = 1.0
            arrow.color.g = 0.6
            arrow.color.b = 0.0
            arrow.color.a = 0.95

            if i == 0:
                robot_pose = self._GetRobotPose(frame)
                if robot_pose is None:
                    arrow.pose.position = poses[0].pose.position
                    yaw = 0.0
                else:
                    rx, ry, _ = robot_pose
                    arrow.pose.position.x = rx
                    arrow.pose.position.y = ry
                    curr = poses[0].pose.position
                    yaw = math.atan2(curr.y - ry, curr.x - rx)
            else:
                prev = poses[i - 1].pose.position
                curr = poses[i].pose.position
                arrow.pose.position = prev
                yaw = math.atan2(curr.y - prev.y, curr.x - prev.x)

            qx, qy, qz, qw = yaw_to_quat(yaw)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            ma.markers.append(arrow)

        self._last_marker_count = n
        self.vis_pub.publish(ma)

    def _ClearPreview(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        m_line = Marker()
        m_line.header.frame_id = DEFAULT_FRAME
        m_line.header.stamp = now
        m_line.ns = 'waypoints'
        m_line.id = 0
        m_line.action = Marker.DELETE
        ma.markers.append(m_line)

        for i in range(self._last_marker_count):
            m = Marker()
            m.header.frame_id = DEFAULT_FRAME
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = 100 + i
            m.action = Marker.DELETE
            ma.markers.append(m)

        self._last_marker_count = 0
        self.vis_pub.publish(ma)

    # Requests
    def request_send(self):
        self._send_requested = True

    def request_load_and_send(self):
        self._load_send_requested = True

    def _PollRequests(self):
        if self._send_requested:
            self._send_requested = False
            self._SendCurrentWaypointsAsync()
        if self._load_send_requested:
            self._load_send_requested = False
            self._LoadAndMaybeLoop()

    # Save / Load
    def _SaveCurrentWaypoints(self):
        with self._lock:
            poses = list(self._poses)
        if len(poses) == 0:
            self.get_logger().warn("No waypoints to save.")
            return
        data = [{"x": p.pose.position.x, "y": p.pose.position.y, "frame_id": (p.header.frame_id or DEFAULT_FRAME)} for p in poses]
        try:
            with open(SAVE_PATH, "w") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Saved {len(poses)} waypoints to {SAVE_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")

    def _LoadPositions(self) -> Optional[List[PoseStamped]]:
        if not os.path.exists(SAVE_PATH):
            self.get_logger().warn(f"No saved file found at {SAVE_PATH}")
            return None
        try:
            with open(SAVE_PATH, "r") as f:
                arr = json.load(f)
            poses: List[PoseStamped] = []
            for item in arr:
                ps = PoseStamped()
                ps.header.frame_id = str(item.get("frame_id", DEFAULT_FRAME))
                ps.pose.position.x = float(item["x"])
                ps.pose.position.y = float(item["y"])
                ps.pose.orientation.w = 1.0
                poses.append(ps)
            return poses
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return None

    # Send helpers
    def _SendPosesAsync(self, base_poses: List[PoseStamped]):
        if not base_poses:
            self.get_logger().warn("No waypoints to send.")
            return
        if not self.cli.wait_for_server(timeout_sec=0.5):
            self.get_logger().error("No FollowWaypoints server! Is nav2_waypoint_follower running?")
            return

        yaws = self._ComputeYawsForSend(base_poses)
        poses_with_yaw = self._ApplyYawsToPoses(base_poses, yaws)

        goal = FollowWaypoints.Goal()
        goal.poses = poses_with_yaw

        self._goal_active = True
        self.get_logger().info(f"Sending {len(poses_with_yaw)} waypoints ...")
        send_future = self.cli.send_goal_async(goal)
        send_future.add_done_callback(self._OnGoalResponse)

    def _SendCurrentWaypointsAsync(self):
        with self._lock:
            base_poses = list(self._poses)
        self._SendPosesAsync(base_poses)

    # Loop scheduling
    def _ScheduleNextLoop(self):
        self._StopLoopTimer()
        period = float(LOOP_INTERVAL_MS) / 1000.0

        def _cb():
            try:
                self._loop_timer.cancel()
            except Exception:
                pass
            self._loop_timer = None

            if not self.LOOP_FLAG or self._goal_active or not self._loop_poses_raw:
                return

            if self._pingpong_reverse_next:
                route = self._BuildLapPoses(self._loop_poses_raw, forward=False)
                mode = "REVERSE"
            else:
                route = self._BuildLapPoses(self._loop_poses_raw, forward=True)
                mode = "FORWARD"

            if not route:
                self.get_logger().warn("No valid route to send in loop.")
                return

            self.get_logger().info(f"Loop delay elapsed ({LOOP_INTERVAL_MS} ms). Re-sending waypoints in {mode} order ...")
            self._pingpong_reverse_next = not self._pingpong_reverse_next
            self._SendPosesAsync(route)

        self._loop_timer = self.create_timer(period, _cb)

    def _StopLoopTimer(self):
        if self._loop_timer is not None:
            try:
                self._loop_timer.cancel()
            except Exception:
                pass
            self._loop_timer = None

    def _LoadAndMaybeLoop(self):
        loaded = self._LoadPositions()
        if not loaded:
            return

        with self._lock:
            self._poses = loaded
        self._PublishPreview()

        self._loop_poses_raw = loaded
        self._StopLoopTimer()
        self._pingpong_reverse_next = True

        route = self._BuildLapPoses(self._loop_poses_raw, forward=True)
        if not route:
            self.get_logger().warn("Loaded route is empty after near-first filtering; nothing to send.")
            return

        if self.LOOP_FLAG:
            self.get_logger().info(f"LOOP_FLAG=True: First lap FORWARD starts now, next lap will be REVERSE after {LOOP_INTERVAL_MS} ms.")
        self._SendPosesAsync(route)

    # Action callbacks
    def _OnGoalResponse(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error("FollowWaypoints goal rejected.")
            return
        self.get_logger().info("Goal accepted. Waiting for result ...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._OnResult)

    def _OnResult(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self._goal_active = False
            self.get_logger().error(f"Failed to get result: {e}")
            return

        self._goal_active = False

        if hasattr(result, "missed_waypoints") and result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {list(result.missed_waypoints)}")
        else:
            self.get_logger().info("All waypoints completed successfully!")

        if self.LOOP_FLAG and self._loop_poses_raw:
            self._ScheduleNextLoop()
            return

        with self._lock:
            self._poses.clear()
        self._ClearPreview()
        self.get_logger().info("Ready for new waypoints. Use RViz2 Publish Point and press Enter.")

# Main
def main():
    rclpy.init()
    node = WaypointCollector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        with RawKeyReader() as kr:
            while True:
                key = kr.readkey()
                if key in (KEY_ENTER_CR, KEY_ENTER_LF):
                    node.request_send()
                elif key == KEY_F8:
                    node.get_logger().info("Saving current waypoints ...")
                    node._SaveCurrentWaypoints()
                elif key == KEY_F7:
                    node.get_logger().info("Loading saved waypoints and sending ...")
                    node.request_load_and_send()
                elif key == KEY_F6:
                    node.LOOP_FLAG = not node.LOOP_FLAG
                    node.get_logger().info(f"LOOP_FLAG: {'TRUE' if node.LOOP_FLAG else 'FALSE'}")
                    if not node.LOOP_FLAG:
                        node._StopLoopTimer()
                elif key == KEY_CTRL_C:
                    node.get_logger().info("Ctrl+C pressed, exiting ...")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node._StopLoopTimer()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=0.5)

if __name__ == '__main__':
    main()
