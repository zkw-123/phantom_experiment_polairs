#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import argparse
import datetime
from scipy import fftpack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class UltrasoundReader:
    def __init__(self, mask_path=None, device_id=6):
        self.device_id = device_id
        self.mask = None
        
        # 加载掩码（如果提供）
        if mask_path and os.path.exists(mask_path):
            self.mask = np.load(mask_path)
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.device_id)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video device {self.device_id}")
    
    def capture_frame(self):
        """捕获一帧超声图像"""
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def process_frame(self, frame):
        """处理超声图像，提取ROI和计算质量指标"""
        if self.mask is None:
            return frame, frame, [0.0, 0.0, 0.0]
        
        # 提取ROI
        roi_img = self.img_sorter(frame, self.mask)
        
        # 计算质量指标
        ct, cv_val, cl = self.calc_image_indices(frame, self.mask)
        
        return frame, roi_img, [ct, cv_val, cl]
    
    def close(self):
        """关闭摄像头"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
    
    # 以下是用于图像处理的辅助方法
    
    def img_sorter(self, img, map):
        """从图像中提取ROI"""
        temp = np.zeros_like(img, dtype=np.uint8)
        idx = np.argwhere(map == 1)
        B, G, R = cv2.split(img)
        temp[:,:,2] = np.multiply(R, map)
        temp[:,:,1] = np.multiply(G, map)
        temp[:,:,0] = np.multiply(B, map)

        iH_min = int(np.min(idx[:,0]))
        iW_min = int(np.min(idx[:,1]))
        iH_max = int(np.max(idx[:,0]))+1
        iW_max = int(np.max(idx[:,1]))+1

        result = temp[iH_min:iH_max, iW_min:iW_max].copy()
        return result

    def binary_process(self, img, ksize):
        """对图像应用二值处理"""
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        opn_img = cv2.morphologyEx(img, cv2.MORPH_OPEN, k)
        _, bin_img = cv2.threshold(opn_img, 0, 255, cv2.THRESH_OTSU)
        return bin_img

    def calc_image_indices(self, in_img, in_map):
        """计算图像质量指标：对比度、变异系数和清晰度"""
        # 初始化参数:
        gray_img = cv2.cvtColor(in_img, cv2.COLOR_BGR2GRAY)  
        bin_img = self.binary_process(gray_img, 5)
        po_list = np.argwhere(in_map == 1)
        temp = gray_img[po_list[:,0], po_list[:,1]]

        # 计算对比度:
        w_vals = gray_img[po_list[:,0], po_list[:,1]][bin_img[po_list[:,0], po_list[:,1]] == 255]
        b_vals = gray_img[po_list[:,0], po_list[:,1]][bin_img[po_list[:,0], po_list[:,1]] == 0]
        w_mean = np.mean(w_vals) if len(w_vals) > 0 else 0
        b_mean = np.mean(b_vals) if len(b_vals) > 0 else 0
        ct_result = round(b_mean-w_mean, 2)

        # 计算变异系数:
        mean_val = np.mean(temp)
        std_val = np.std(temp)
        cv_result = std_val/mean_val if mean_val > 0 else 0

        # 计算清晰度:
        img_fft = fftpack.fft(temp)
        img_spectrum = np.abs(img_fft)
        img_spectrum /= np.max(img_spectrum) if np.max(img_spectrum) > 0 else 1
        hist, _ = np.histogram(img_spectrum, bins=256, range=(0,1))
        pixel_num = hist.sum()
        mean = (hist * np.arange(256)).sum() / pixel_num if pixel_num > 0 else 0
        stddev = np.sqrt((hist * (np.arange(256) - mean)**2).sum() / pixel_num) if pixel_num > 0 else 0
        cl_result = stddev / mean if mean > 0 else 0

        return ct_result, cv_result, cl_result


class ROS2UltrasoundReaderNode(Node):
    def __init__(self, mask_path=None, device_id=0, sample_rate=30, save_path="/ros2_ws/ultrasound_data", display=False, save_images=False, save_video=False):
        super().__init__('ros2_ultrasound_reader')
        
        # 参数声明
        self.declare_parameter('display', display)
        self.declare_parameter('save_images', save_images)
        self.declare_parameter('save_video', save_video)
        
        # 获取参数
        self.display = self.get_parameter('display').value
        self.save_images = self.get_parameter('save_images').value
        self.save_video = self.get_parameter('save_video').value
        
        # 初始化日志
        self.get_logger().info(f"Initializing ultrasound reader with device {device_id}")
        
        # 创建会话时间戳
        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 设置保存路径
        self.save_path = save_path
        if self.save_images or self.save_video:
            # 创建以会话时间戳命名的子目录
            self.session_dir = os.path.join(save_path, self.session_timestamp)
            os.makedirs(self.session_dir, exist_ok=True)
            self.get_logger().info(f"Saving ultrasound data to: {self.session_dir}")
        
        # 创建UltrasoundReader实例
        try:
            self.ultrasound_reader = UltrasoundReader(mask_path, device_id)
            self.get_logger().info("Ultrasound reader initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ultrasound reader: {str(e)}")
            raise
        
        # 设置发布者
        self.image_publisher = self.create_publisher(Image, 'us_img', 10)
        self.roi_publisher = self.create_publisher(Image, 'us_roi', 10)
        self.indices_publisher = self.create_publisher(Float32MultiArray, 'us_idx', 10)
        
        # 创建CV Bridge
        self.bridge = CvBridge()
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / sample_rate, self.timer_callback)
        
        # 帧计数器
        self.frame_counter = 0
        
        # 初始化视频写入器（如果需要）
        self.video_writer = None
        if self.save_video:
            try:
                # 获取视频参数
                width = int(self.ultrasound_reader.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(self.ultrasound_reader.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = sample_rate
                
                # 创建视频文件
                video_path = os.path.join(self.session_dir, f"ultrasound_{self.session_timestamp}.avi")
                self.get_logger().info(f"Saving video to: {video_path}")
                
                # 创建视频写入器
                fourcc = cv2.VideoWriter_fourcc(*'DIVX')
                self.video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            except Exception as e:
                self.get_logger().error(f"Failed to initialize video writer: {str(e)}")
        
        self.get_logger().info("ROS2 Ultrasound Reader initialized")
        self.get_logger().info("Publishing ultrasound data to 'us_img', 'us_roi', and 'us_idx' topics")
        if self.display:
            self.get_logger().info("Display enabled: Press 'q' to quit, 's' to save current frame")
    
    def timer_callback(self):
        """定时器回调，捕获并处理超声图像"""
        try:
            # 获取当前时间戳
            now = self.get_clock().now().to_msg()
            timestamp = now.sec + now.nanosec / 1e9
            
            # 捕获超声图像
            frame = self.ultrasound_reader.capture_frame()
            
            if frame is None:
                self.get_logger().warn("Failed to capture frame")
                return
            
            # 处理图像
            full_img, roi_img, indices = self.ultrasound_reader.process_frame(frame)
            
            # 记录质量指标
            self.get_logger().debug(f'[Idxs] Ct:{indices[0]:.2f}, Cv:{indices[1]:.2f}, Cl:{indices[2]:.2f}')
            
            # 保存图像（如果启用）
            if self.save_images:
                # 创建文件名，包含序列号和精确时间戳
                image_filename = f"us_{self.frame_counter:06d}_{timestamp:.6f}.png"
                image_path = os.path.join(self.session_dir, image_filename)
                cv2.imwrite(image_path, full_img)
                
                # 也可以保存ROI图像
                roi_filename = f"roi_{self.frame_counter:06d}_{timestamp:.6f}.png"
                roi_path = os.path.join(self.session_dir, roi_filename)
                cv2.imwrite(roi_path, roi_img)
            
            # 保存视频帧（如果启用）
            if self.video_writer is not None:
                try:
                    self.video_writer.write(full_img)
                except Exception as e:
                    self.get_logger().warn(f"Could not write to video: {str(e)}")
            
            # 发布数据
            self.publish_data(full_img, roi_img, indices, timestamp)
            
            # 显示图像（如果启用）
            if self.display:
                try:
                    # 在原始图像上显示质量指标
                    display_img = full_img.copy()
                    cv2.putText(display_img, f"Ct: {indices[0]:.2f}, Cv: {indices[1]:.2f}, Cl: {indices[2]:.2f}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # 显示图像
                    cv2.imshow('Ultrasound Image', display_img)
                    
                    # 如果有ROI，也显示ROI图像
                    if roi_img is not None and roi_img.size > 0:
                        cv2.imshow('ROI Image', roi_img)
                    
                    # 处理键盘事件
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info("User requested exit")
                        # 退出ROS2节点
                        rclpy.shutdown()
                        return
                    elif key == ord('s'):
                        # 手动保存当前帧
                        save_timestamp = self.get_clock().now().to_msg().sec
                        save_path = os.path.join(self.session_dir, f"manual_save_{save_timestamp}.png")
                        cv2.imwrite(save_path, full_img)
                        self.get_logger().info(f"Manually saved frame to {save_path}")
                except Exception as e:
                    self.get_logger().warn(f"Could not display image: {str(e)}")
            
            self.frame_counter += 1
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
    
    def publish_data(self, full_img, roi_img, indices, timestamp):
        """发布超声图像和质量指标到ROS话题"""
        try:
            # 创建图像消息
            img_msg = self.bridge.cv2_to_imgmsg(full_img, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()  # 添加时间戳
            
            roi_msg = self.bridge.cv2_to_imgmsg(roi_img, "bgr8")
            roi_msg.header.stamp = self.get_clock().now().to_msg()  # 添加时间戳
            
            # 创建质量指标消息
            idx_msg = Float32MultiArray()
            idx_msg.data = indices
            
            # 发布消息
            self.image_publisher.publish(img_msg)
            self.roi_publisher.publish(roi_msg)
            self.indices_publisher.publish(idx_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing data: {str(e)}")
    
    def destroy_node(self):
        """清理资源"""
        try:
            # 关闭视频写入器
            if hasattr(self, 'video_writer') and self.video_writer is not None:
                self.video_writer.release()
                self.get_logger().info("Video writer closed")
            
            # 关闭超声读取器
            if hasattr(self, 'ultrasound_reader'):
                self.ultrasound_reader.close()
                self.get_logger().info("Ultrasound reader closed")
            
            # 关闭所有OpenCV窗口
            cv2.destroyAllWindows()
            
        except Exception as e:
            self.get_logger().error(f"Error closing resources: {str(e)}")
        
        super().destroy_node()


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--mask_path', help="Path to the ROI mask file (.npy)")
    parser.add_argument('--device_id', help="Video device ID", type=int, default=0)
    parser.add_argument('--sample_rate', help="Sample rate in Hz", type=float, default=30.0)
    parser.add_argument('--save_path', help="Path to save images and videos", default="/ros2_ws/ultrasound_data")
    parser.add_argument('--display', help="Enable display window", action='store_true')
    parser.add_argument('--save_images', help="Enable image saving", action='store_true')
    parser.add_argument('--save_video', help="Enable video saving", action='store_true')
    
    # 解析参数
    parsed_args, remaining = parser.parse_known_args()
    
    try:
        # 创建并初始化节点
        node = ROS2UltrasoundReaderNode(
            mask_path=parsed_args.mask_path,
            device_id=parsed_args.device_id,
            sample_rate=parsed_args.sample_rate,
            save_path=parsed_args.save_path,
            display=parsed_args.display,
            save_images=parsed_args.save_images,
            save_video=parsed_args.save_video
        )
        
        # 保持节点运行
        rclpy.spin(node)
        
    except Exception as e:
        print(f"Error in ROS2 Ultrasound Reader: {str(e)}")
    finally:
        # 清理
        rclpy.shutdown()


if __name__ == '__main__':
    main()