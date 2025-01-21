#!/bin/bash

# 检查ROS环境
if [ -z "$ROS_ROOT" ]; then
    echo "ROS环境未设置,请先source ROS的setup.bash"
    exit 1
fi

# 检查参数
if [ "$#" -ne 1 ]; then
    echo "使用方法: $0 <bag文件路径>"
    echo "例如: $0 output/sensor_data_20250117_152951.bag"
    exit 1
fi

BAG_FILE=$1

# 检查文件是否存在
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag文件 '$BAG_FILE' 不存在"
    exit 1
fi

# 定义清理函数
cleanup() {
    echo "正在关闭所有节点..."
    pkill -P $$
    killall -9 rviz 2>/dev/null
    exit 0
}

# 捕获Ctrl+C信号
trap cleanup SIGINT SIGTERM

# 显示bag信息
echo "正在分析bag文件信息..."
rosbag info $BAG_FILE

# 启动rviz
echo "正在启动rviz..."
rosrun rviz rviz -d sensor.rviz &
sleep 2

# 创建新终端显示图像
echo "正在启动图像查看器..."
gnome-terminal -- bash -c "rosrun image_view image_view image:=/zed2/zed_node/rgb/image_rect_color" &
sleep 1

# 创建新终端显示深度图
echo "正在启动深度图查看器..."
gnome-terminal -- bash -c "rosrun image_view image_view image:=/zed2/zed_node/depth/depth_registered" &
sleep 1

# 播放bag文件
echo "开始播放bag文件..."
echo "提示: 空格键 - 暂停/继续"
echo "      S键 - 步进模式"
echo "      右箭头 - 加速"
echo "      左箭头 - 减速"
echo "按Ctrl+C停止播放"

rosbag play $BAG_FILE --clock \
    /zed2/zed_node/rgb/image_rect_color \
    /zed2/zed_node/depth/depth_registered \
    /zed2/zed_node/point_cloud/cloud_registered \
    /rslidar_points \
    /sensor_imu

# 等待播放完成或Ctrl+C
wait

