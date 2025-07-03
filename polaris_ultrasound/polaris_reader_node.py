#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
import sys
import numpy as np
import argparse
import threading
import json
from std_msgs.msg import String
from sksurgerynditracker.nditracker import NDITracker

class PolarisReader:
    def __init__(self, rom_paths):
        self.rom_paths = rom_paths
        self.SETTINGS = {
            "tracker type": "polaris",
            "romfiles": self.rom_paths,
            "serial port": "/dev/ttyUSB0"
        }
        self.TRACKER = NDITracker(self.SETTINGS)

    def start(self):
        self.TRACKER.start_tracking()

    def stop(self):
        self.TRACKER.stop_tracking()
        self.TRACKER.close()

    def get_frame(self):
        return self.TRACKER.get_frame()

class ROS2PolarisReaderNode(Node):
    def __init__(self, rom_paths, tool_names):
        super().__init__('ros2_polaris_reader')

        # 自动填充 tool_names 不足
        if len(tool_names) < len(rom_paths):
            for i in range(len(tool_names), len(rom_paths)):
                tool_names.append(f"tool{i}")

        self.tool_names = tool_names
        self.rom_paths = rom_paths
        self.polaris_reader = PolarisReader(rom_paths)

        self.publisher = self.create_publisher(String, 'ndi_transforms', 10)
        self.frame_counter = 0

        self.get_logger().info("ROS2 Polaris Reader initialized")
        self.get_logger().info(f"Using ROM files: {self.rom_paths}")
        self.get_logger().info(f"Tool names: {self.tool_names}")
        self.get_logger().info("Publishing tracking data to 'ndi_transforms' topic")

    def run(self):
        self.get_logger().info("Starting Polaris tracking...")
        thread = threading.Thread(target=self.tracking_loop, daemon=True)
        thread.start()

    def tracking_loop(self):
        try:
            self.polaris_reader.start()
            while rclpy.ok():
                port_handles, timestamps, framenumbers, trackings, qualities = self.polaris_reader.get_frame()

                for i, timestamp in enumerate(timestamps):
                    print(f"[{i}] Tool: {self.tool_names[i] if i < len(self.tool_names) else f'tool{i}'}")
                    print(f"Timestamp: {timestamp}")
                    print(trackings[i])

                self.publish_tracking_data(port_handles, timestamps, framenumbers, trackings, qualities)
                self.frame_counter += 1
        except Exception as e:
            self.get_logger().error(f"Error in tracking loop: {str(e)}")
        finally:
            try:
                self.polaris_reader.stop()
                self.get_logger().info("Polaris tracking stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping tracker: {str(e)}")

    def publish_tracking_data(self, port_handles, timestamps, framenumbers, trackings, qualities):
        try:
            now = self.get_clock().now().to_msg()
            current_timestamp = now.sec + now.nanosec / 1e9

            transforms_data = []
            for i, tracking in enumerate(trackings):
                if i < len(port_handles):
                    matrix = tracking.tolist() if isinstance(tracking, np.ndarray) else tracking
                    transform_info = {
                        "tool_id": int(port_handles[i]) if isinstance(port_handles[i], (int, np.integer)) else str(port_handles[i]),
                        "tool_name": self.tool_names[i] if i < len(self.tool_names) else f"tool{i}",
                        "quality": float(qualities[i]) if i < len(qualities) else 0.0,
                        "matrix": matrix
                    }
                    if isinstance(tracking, np.ndarray) and tracking.shape == (4, 4):
                        transform_info["translation"] = tracking[:3, 3].tolist()
                    transforms_data.append(transform_info)

            data = {
                "timestamp": current_timestamp,
                "original_timestamp": str(timestamps[0]) if timestamps else "",
                "frame_number": self.frame_counter,
                "transforms": transforms_data
            }

            msg = String()
            msg.data = json.dumps(data)
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing tracking data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ros2_polaris_reader_temp')
    rom_path_str = node.declare_parameter('rom_path', '').get_parameter_value().string_value
    tool_names_str = node.declare_parameter('tool_names', '').get_parameter_value().string_value
    node.destroy_node()  # 释放这个临时节点

    rom_paths = rom_path_str.split(',') if rom_path_str else []
    tool_names = tool_names_str.split(',') if tool_names_str else []

    if len(rom_paths) == 0:
        print("❗ Error: At least one rom_path must be provided (comma-separated)")
        rclpy.shutdown()
        sys.exit(1)

    try:
        node = ROS2PolarisReaderNode(rom_paths, tool_names)
        node.run()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
