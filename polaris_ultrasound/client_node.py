#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time

class NDITrackerClient(Node):
    """
    Example ROS 2 client node to communicate with the NDI tracker node.
    
    This node demonstrates:
    1. Subscribing to transform data from the NDI tracker
    2. Sending commands to the NDI tracker service
    """
    
    def __init__(self):
        super().__init__('ndi_tracker_client')
        
        # Subscribe to transform data
        self.subscription = self.create_subscription(
            String,
            'ndi_transforms',
            self.transform_callback,
            10)
        
        # Client for the command service
        self.command_client = self.create_client(Trigger, 'ndi_command')
        
        # Wait for service to be available
        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for NDI tracker service...')
        
        self.get_logger().info('Connected to NDI tracker service')
        
        # Examples of commands you can send (uncomment to use)
        # self.send_command("start_logging")
        # self.get_status()
    
    def transform_callback(self, msg):
        """
        Process transform data received from the NDI tracker.
        """
        try:
            data = json.loads(msg.data)
            timestamp = data.get("timestamp", "")
            transforms = data.get("transforms", [])
            
            self.get_logger().info(f'Received transforms at {timestamp}')
            
            # Process each transform
            for t in transforms:
                tool_id = t.get("tool_id")
                quality = t.get("quality")
                
                # Access matrix, quaternion, or translation based on what's available
                if "matrix" in t:
                    matrix = t.get("matrix")
                    self.get_logger().info(f'Tool {tool_id} matrix: {matrix}')
                
                if "translation" in t:
                    translation = t.get("translation")
                    self.get_logger().info(f'Tool {tool_id} position: {translation}')
                
                if "quaternion" in t:
                    quaternion = t.get("quaternion")
                    self.get_logger().info(f'Tool {tool_id} orientation (quaternion): {quaternion}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing transform data: {str(e)}')
    
    def send_command(self, command):
        """
        Send a command to the NDI tracker service.
        
        Commands:
        - "start_logging": Start recording transform data to a log file
        - "stop_logging": Stop recording
        - "get_status": Get the current status of the NDI tracker
        """
        request = Trigger.Request()
        # For Trigger service, we can't pass the command directly,
        # so we put it in the node's name for the service to detect
        future = self.command_client.call_async(request)
        future.add_done_callback(
            lambda f: self.command_callback(f, command))
    
    def command_callback(self, future, command):
        """Process the result of a command."""
        try:
            response = future.result()
            self.get_logger().info(f'Command "{command}" result: {response.success}')
            self.get_logger().info(f'Response message: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    
    def get_status(self):
        """Get the current status of the NDI tracker."""
        self.send_command("get_status")


def main(args=None):
    rclpy.init(args=args)
    client_node = NDITrackerClient()
    
    try:
        rclpy.spin(client_node)
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
