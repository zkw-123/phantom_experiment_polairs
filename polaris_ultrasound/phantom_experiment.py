#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import datetime
from std_msgs.msg import String
import threading

class StylusPointSaver(Node):
    def __init__(self):
        super().__init__('stylus_point_saver')

        self.declare_parameter('save_path', '/ros2_ws/calibration_data/polaris/stylus_phantom')
        self.declare_parameter('stylus_name', 'stylus')
        self.declare_parameter('phantom_name', 'phantom')

        self.save_path = self.get_parameter('save_path').value
        self.stylus_name = self.get_parameter('stylus_name').value
        self.phantom_name = self.get_parameter('phantom_name').value

        os.makedirs(self.save_path, exist_ok=True)

        self.latest_transforms = {}  # tool_name → {'matrix': ..., 'translation': ..., 'quality': ...}
        self.latest_timestamp = 0.0

        self.subscription = self.create_subscription(String, 'ndi_transforms', self.ndi_callback, 10)

        self.get_logger().info("StylusPointSaver initialized. Type point name and press Enter to save.")

        # Start input listener
        threading.Thread(target=self.input_loop, daemon=True).start()

    def ndi_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_timestamp = float(data.get('timestamp', 0.0))
            self.latest_transforms.clear()

            for t in data.get('transforms', []):
                tool = t.get('tool_name', '')
                if tool in [self.stylus_name, self.phantom_name]:
                    self.latest_transforms[tool] = {
                        'matrix': t.get('matrix', []),
                        'translation': t.get('translation', []),
                        'quality': t.get('quality', 0.0)
                    }

        except Exception as e:
            self.get_logger().error(f"Failed to parse Polaris message: {e}")

    def input_loop(self):
        while rclpy.ok():
            point_name = input("Enter point name (e.g., point1): ").strip()
            if point_name.lower() == 'exit':
                print("Exiting input loop.")
                break
            if not point_name:
                print("Point name cannot be empty.")
                continue
            self.save_point_data(point_name)

    def save_point_data(self, point_name):
        # Check availability
        if self.stylus_name not in self.latest_transforms:
            self.get_logger().warn("Stylus data not available.")
            return
        if self.phantom_name not in self.latest_transforms:
            self.get_logger().warn("Phantom data not available.")
            return

        timestamp = self.latest_timestamp
        timestamp_str = f"{timestamp:.6f}"

        filename = f"{point_name}_{timestamp_str}.json"
        filepath = os.path.join(self.save_path, filename)

        save_data = {
            "point_name": point_name,
            "timestamp_ros": timestamp,
            "stylus": self.latest_transforms[self.stylus_name],
            "phantom": self.latest_transforms[self.phantom_name]
        }

        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2)

        self.get_logger().info(f"Saved point '{point_name}' to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = StylusPointSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()