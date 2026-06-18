#!/usr/bin/env python3
"""Quick camera viewer — shows D435i head + D405 wrist side by side.

Usage:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 camera_viewer.py

Press 'q' to quit.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.head_img = None
        self.wrist_img = None

        self.head_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._head_cb, 10)
        # Try both possible topic names for wrist camera
        self.wrist_sub = self.create_subscription(
            Image, '/right_wrist_camera/color/image_raw', self._wrist_cb, 10)
        self.wrist_sub2 = self.create_subscription(
            Image, '/camera/right_wrist_camera/color/image_raw', self._wrist_cb, 10)

        self.timer = self.create_timer(0.033, self._display)  # 30 Hz
        self.get_logger().info('Camera viewer started. Press q to quit.')
        self.get_logger().info('  Head cam:  /camera/color/image_raw')
        self.get_logger().info('  Wrist cam: /right_wrist_camera/color/image_raw')

    def _head_cb(self, msg):
        self.head_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _wrist_cb(self, msg):
        self.wrist_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _display(self):
        # Create display frames
        h, w = 480, 640

        if self.head_img is not None:
            head = cv2.resize(self.head_img, (w, h))
            cv2.putText(head, 'HEAD (D435i)', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            head = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(head, 'HEAD: waiting...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if self.wrist_img is not None:
            wrist = cv2.resize(self.wrist_img, (w, h))
            cv2.putText(wrist, 'R_WRIST (D405)', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            wrist = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(wrist, 'WRIST: waiting...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        combined = np.hstack([head, wrist])
        cv2.imshow('OpenArm Cameras (press q to quit)', combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
