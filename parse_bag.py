#!/usr/bin/env python3
import rosbag
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import sensor_msgs.point_cloud2 as pc2
import argparse
from tqdm import tqdm
import yaml

class BagExtractor:
    def __init__(self, bag_path, output_dir):
        """初始化提取器"""
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.bridge = CvBridge()
        
        # 创建输出目录
        self.dirs = {
            'image_left': os.path.join(output_dir, 'images_left'),
            'image_right': os.path.join(output_dir, 'images_right'),
            'depth': os.path.join(output_dir, 'depth'),
            'lidar_left': os.path.join(output_dir, 'lidar_left'),
            'lidar_right': os.path.join(output_dir, 'lidar_right'),
            'imu': os.path.join(output_dir, 'imu'),
            'gps': os.path.join(output_dir, 'gps')
        }
        self.create_directories()
        
        # 初始化数据记录字典
        self.timestamps = {
            'image_left': [],
            'image_right': [],
            'depth': [],
            'lidar_left': [],
            'lidar_right': [],
            'imu': [],
            'gps': []
        }

    def create_directories(self):
        """创建必要的目录结构"""
        for dir_path in self.dirs.values():
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)

    def save_image(self, msg, timestamp, image_type='image'):
        """保存图像数据"""
        try:
            if image_type == 'depth':
                # 深度图像通常是32FC1格式
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                # 将深度值归一化到0-65535范围内保存为16位PNG
                cv_img = (cv_img * 1000).astype(np.uint16)  # 转换为毫米单位
                filename = os.path.join(self.dirs[image_type], f'{timestamp}.png')
                cv2.imwrite(filename, cv_img)
            else:
                # 普通图像
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = os.path.join(self.dirs[image_type], f'{timestamp}.jpg')
                cv2.imwrite(filename, cv_img)
            
            self.timestamps[image_type].append(timestamp)
            
        except Exception as e:
            print(f"Error saving {image_type}: {e}")

    def save_pointcloud(self, msg, timestamp):
        """保存点云数据为PCD格式"""
        try:
            # 将ROS消息转换为点云数据
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points.append(point)
            
            points = np.array(points)
            
            # 保存为PCD文件
            filename = os.path.join(self.dirs['lidar'], f'{timestamp}.pcd')
            
            # 写入PCD文件头
            with open(filename, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data\n")
                f.write("VERSION 0.7\n")
                f.write(f"FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                
                # 写入点云数据
                for point in points:
                    f.write(f"{point[0]} {point[1]} {point[2]} {point[3]}\n")
            
            self.timestamps['lidar'].append(timestamp)
            
        except Exception as e:
            print(f"Error saving pointcloud: {e}")

    def save_imu(self, msg, timestamp):
        """保存IMU数据为CSV格式"""
        try:
            filename = os.path.join(self.dirs['imu'], f'{timestamp}.csv')
            with open(filename, 'w') as f:
                f.write("timestamp,ax,ay,az,wx,wy,wz\n")
                f.write(f"{timestamp},{msg.linear_acceleration.x},{msg.linear_acceleration.y},"
                       f"{msg.linear_acceleration.z},{msg.angular_velocity.x},{msg.angular_velocity.y},"
                       f"{msg.angular_velocity.z}\n")
            
            self.timestamps['imu'].append(timestamp)
            
        except Exception as e:
            print(f"Error saving IMU: {e}")

    def save_gps(self, msg, timestamp):
        """保存GPS数据为CSV格式"""
        try:
            filename = os.path.join(self.dirs['gps'], f'{timestamp}.csv')
            with open(filename, 'w') as f:
                f.write("timestamp,latitude,longitude,altitude\n")
                f.write(f"{timestamp},{msg.latitude},{msg.longitude},{msg.altitude}\n")
            
            self.timestamps['gps'].append(timestamp)
            
        except Exception as e:
            print(f"Error saving GPS: {e}")

    def save_timestamps(self):
        """保存时间戳信息"""
        for sensor_type, timestamps in self.timestamps.items():
            if timestamps:
                filename = os.path.join(self.output_dir, f'{sensor_type}_timestamps.txt')
                with open(filename, 'w') as f:
                    for ts in timestamps:
                        f.write(f"{ts}\n")

    def extract(self):
        """提取bag文件中的所有数据"""
        print(f"Opening bag file: {self.bag_path}")
        bag = rosbag.Bag(self.bag_path)
        
        # 获取消息总数用于进度条
        total_msgs = bag.get_message_count()
        
        # 使用tqdm创建进度条
        with tqdm(total=total_msgs, desc="Extracting data") as pbar:
            for topic, msg, t in bag.read_messages():
                timestamp = str(t.to_nsec())  # 使用纳秒级时间戳
                
                # 根据话题类型分别处理
                if msg._type == "sensor_msgs/Image":
                    if "depth" in topic:
                        self.save_image(msg, timestamp, 'depth')
                    else:
                        self.save_image(msg, timestamp, 'image')
                elif msg._type == "sensor_msgs/PointCloud2":
                    self.save_pointcloud(msg, timestamp)
                elif msg._type == "sensor_msgs/Imu":
                    self.save_imu(msg, timestamp)
                elif msg._type == "sensor_msgs/NavSatFix":
                    self.save_gps(msg, timestamp)
                
                pbar.update(1)
        
        # 保存时间戳信息
        self.save_timestamps()
        bag.close()
        print("Extraction completed!")

def main():
    parser = argparse.ArgumentParser(description="Extract data from ROS bag file")
    parser.add_argument("bag_path", help="Path to the ROS bag file")
    parser.add_argument("--output_dir", default="extracted_data", help="Output directory for extracted data")
    args = parser.parse_args()

    extractor = BagExtractor(args.bag_path, args.output_dir)
    extractor.extract()

if __name__ == "__main__":
    main()
