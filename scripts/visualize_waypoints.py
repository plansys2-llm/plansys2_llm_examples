#!/usr/bin/env python3
# Copyright 2026 Álvaro Valencia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Publish waypoints as TF frames in the map frame.

Edit the WAYPOINTS dict below, run the script, and check RViz2 (TF display).

Usage:
  python3 visualize_waypoints.py
  # Edit coords, Ctrl+C, rerun to update
"""

import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ── Edit these coordinates [x, y, theta_rad] ──────────────────────────
WAYPOINTS = {
    "reception":     [ 0.26,  3.66, 0.0],
    "deposit_table": [ 0.07,  0.83, 0.0],
    "shelf_blue":    [-2.03, -1.45, 0.0],
    "shelf_yellow":  [ 0.72, -1.45, 0.0],
    "shelf_green":   [ 0.69, -4.73, 0.0],
    "middle_path":   [ 0.46, -6.40, 0.0],
    "shelf_red":     [-0.77, -1.76, 0.0],
}
# ───────────────────────────────────────────────────────────────────────


class WaypointTFPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        transforms = []
        for name, (x, y, theta) in WAYPOINTS.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'wp_{name}'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            transforms.append(t)

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Published {len(transforms)} waypoint TFs (wp_<name>)")


def main():
    rclpy.init()
    node = WaypointTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
