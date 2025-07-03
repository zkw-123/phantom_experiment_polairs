# calibration_recorder.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import json
import datetime
import shutil
from std_msgs.msg import String

class CalibrationDataRecorder(Node):
    def __init__(self):
        super().__init__('calibration_data_recorder')

        self.declare_parameter('device_id', 6)
        self.declare_parameter('save_path', '/ros2_ws/calibration_data')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('sync_threshold', 0.005)
        self.declare_parameter('tool_names', ['probe', 'phantom'])
        self.declare_parameter('record_time', 60.0)

        self.device_id = self.get_parameter('device_id').value
        self.save_path = self.get_parameter('save_path').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.sync_threshold = self.get_parameter('sync_threshold').value
        self.tool_names = self.get_parameter('tool_names').value
        self.record_time = self.get_parameter('record_time').value

        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        self.ultrasound_dir = os.path.join(self.save_path, "ultrasound", self.session_timestamp)
        self.polaris_dir = os.path.join(self.save_path, "polaris", self.session_timestamp)

        os.makedirs(self.ultrasound_dir, exist_ok=True)
        os.makedirs(self.polaris_dir, exist_ok=True)

        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device {self.device_id}")
            raise RuntimeError(f"Failed to open video device {self.device_id}")

        self.polaris_subscription = self.create_subscription(String, 'ndi_transforms', self.polaris_callback, 10)
        self.polaris_data_queue = []
        self.max_queue_size = 10

        self.frame_counter = 0
        self.polaris_counter = 0

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(1.0 / self.frame_rate, self.timer_callback)

    def polaris_callback(self, msg):
        try:
            data = json.loads(msg.data)
            timestamp_ros = float(data['timestamp'])
            timestamp_hw = float(data.get('original_timestamp', timestamp_ros))

            transforms = {}
            for t in data.get('transforms', []):
                if 'tool_name' in t and 'matrix' in t:
                    tool = t['tool_name']
                    transforms[tool] = {
                        'tool_id': t.get('tool_id', ''),
                        'quality': t.get('quality', 0.0),
                        'matrix': t['matrix'],
                        'translation': t.get('translation', [])
                    }

            self.polaris_data_queue.append({
                'timestamp_ros': timestamp_ros,
                'timestamp_hw': timestamp_hw,
                'transforms': transforms
            })

            if len(self.polaris_data_queue) > self.max_queue_size:
                self.polaris_data_queue.pop(0)

        except Exception as e:
            self.get_logger().error(f"Error in Polaris callback: {str(e)}")

    def find_best_match(self, us_ts):
        best = None
        min_diff = float('inf')
        for item in self.polaris_data_queue:
            diff = abs(us_ts - item['timestamp_ros'])
            if diff < min_diff and diff <= self.sync_threshold:
                min_diff = diff
                best = item
        return best, min_diff

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.start_time > self.record_time:
            rclpy.shutdown()
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        us_ts = self.get_clock().now().nanoseconds / 1e9
        us_filename = f"us_{self.frame_counter:06d}_{us_ts:.6f}.png"
        us_path = os.path.join(self.ultrasound_dir, us_filename)
        cv2.imwrite(us_path, frame)
        self.frame_counter += 1

        best, diff = self.find_best_match(us_ts)
        if not best or not all(tool in best['transforms'] for tool in self.tool_names):
            return

        polaris_filename = f"frame_{self.polaris_counter:06d}_{best['timestamp_hw']:.6f}.json"
        polaris_path = os.path.join(self.polaris_dir, polaris_filename)

        save_data = {
            'timestamp_ros': best['timestamp_ros'],
            'timestamp_hw': best['timestamp_hw'],
            'frame_number': self.polaris_counter,
            'transforms': best['transforms']
        }

        with open(polaris_path, 'w') as f:
            json.dump(save_data, f, indent=2)

        self.polaris_counter += 1

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()