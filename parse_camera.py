#!/usr/bin/env python
import rosbag
import cv2
from cv_bridge import CvBridge
import os
import numpy as np

# 创建保存目录
save_dir = 'extracted_images/raw/left'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 初始化cv_bridge
bridge = CvBridge()

# 打开bag文件
bag = rosbag.Bag('recorded_data/sensor_data_20250226_151653.bag')

# 获取相机话题名称（根据您的bag修改）
camera_topic = '/zed2/zed_node/left_raw/image_raw_color'

# 提取图像并保存
count = 0
for topic, msg, t in bag.read_messages(topics=[camera_topic]):
    try:
        # 将ROS图像消息转换为OpenCV格式
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # 保存图像
        cv2.imwrite(os.path.join(save_dir, f"frame_{count:06d}.jpg"), cv_img)
        print(f"Saved image {count}")
        count += 1
    except Exception as e:
        print(e)

bag.close()
print(f"Total extracted: {count} images")