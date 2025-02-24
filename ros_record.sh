#!/bin/bash

# 检查ROS环境是否已经设置
if [ -z "$ROS_ROOT" ]; then
    echo "ROS环境未设置,请先source ROS的setup.bash"
    echo "例如: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查工作空间是否已经设置
if [ ! -f "devel/setup.bash" ]; then
    echo "未找到工作空间的setup.bash文件"
    echo "请在工作空间根目录运行此脚本"
    exit 1
fi

# source工作空间
source devel/setup.bash

# 创建存储录制数据的目录
DATA_DIR="$(pwd)/recorded_data"
mkdir -p $DATA_DIR

# 获取当前时间作为文件名
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FILENAME="$DATA_DIR/sensor_data_$TIMESTAMP.bag"

echo "开始录制数据..."
echo "数据将保存到: $FILENAME"

# 录制以下话题:
# - ZED相机话题
# - 激光雷达话题
# - IMU话题
# 修改以下话题名称以选择需要录制的话题
# rosbag record -O $FILENAME \
#     /zed2/zed_node/rgb/image_rect_color \
#     /zed2/zed_node/depth/depth_registered \
#     /zed2/zed_node/rgb_raw/image_raw_color \
#     /fix \
#     /sensor_imu \
#     /rslidar_left_points \
#     /rslidar_right_points \
#     __name:=data_recorder


# rosbag record -O $FILENAME \
#     /zed2/zed_node/left/image_rect_color \
#     /zed2/zed_node/right/image_rect_color \
#     /zed2/zed_node/depth/depth_registered \
#     /sensor_imu \
#     /fix \
#     /rslidar_left_packets \
#     /rslidar_right_packets \
#     __name:=data_recorder


# 分别保存了两个相机的图像,可以使用更先进的双目depth算法来求解深度
# 同时保留了自带的深度结果作为参考,confidence_map表示深度图的置信度
rosbag record -O $FILENAME \
    /zed2/zed_node/left_raw/image_raw_color \
    /zed2/zed_node/right_raw/image_raw_color \
    /sensor_imu \
    /fix \
    /rslidar_left_points \
    /rslidar_right_points \
    __name:=data_recorder

# rosbag record -O $FILENAME \
#     /zed2/zed_node/left/image_rect_color \
#     /zed2/zed_node/right/image_rect_color \
#     /zed2/zed_node/right/camera_info \
#     /zed2/zed_node/depth/depth_registered \
#     /zed2/zed_node/depth/camera_info \
#     /zed2/zed_node/confidence/confidence_map \
#     /sensor_imu \
#     /fix \
#     /zed2/zed_node/imu/data_raw \
#     /rslidar_left_packets \
#     /rslidar_right_packets \
#     __name:=data_recorder


# rosbag record -O $FILENAME \
#     /zed2/zed_node/right/image_rect_color \ 
#     __name:=data_recorder


# 直接录制双目的图像
# /zed2/zed_node/stereo/image_rect_color

# rosbag record -O $FILENAME \
#     /zed2/zed_node/disparity/disparity_image \
#     /zed2/zed_node/point_cloud/cloud_registered \
#     /zed2/zed_node/confidence/confidence_map \
#     __name:=data_recorder

# TODO
# 确定是有数据的,但不知道怎么查看这两个话题
# /zed2/zed_node/disparity/disparity_image \
#     /zed2/zed_node/point_cloud/cloud_registered



# 定义清理函数
cleanup() {
    echo "正在停止录制..."
    rosnode kill data_recorder
    exit 0
}

# 捕获Ctrl+C信号
trap cleanup SIGINT SIGTERM

# 等待用户按Ctrl+C
echo "录制中... 按Ctrl+C停止录制"
wait 