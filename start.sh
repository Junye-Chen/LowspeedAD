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

# 定义清理函数
cleanup() {
    echo "正在关闭所有节点..."
    # 先优雅地停止rosbag录制
    if pgrep -x "record" > /dev/null; then
        echo "正在停止数据录制..."
        rosnode kill /data_recorder
        sleep 1
    fi
    # 杀死所有子进程
    pkill -P $$
    # 确保相关进程被终止
    killall -9 rviz roscore rosmaster 2>/dev/null
    exit 0
}

# 捕获Ctrl+C信号
trap cleanup SIGINT SIGTERM

# 启动rslidar_sdk
echo "正在启动rslidar_sdk..."
roslaunch rslidar_sdk start.launch &

# 等待激光雷达节点启动
sleep 2

# 启动ZED相机
echo "正在启动ZED相机..."
roslaunch zed_wrapper zed2.launch &

# 等待所有节点完全启动
sleep 2

# 启动IMU驱动
roslaunch fdilink_ahrs ahrs_data.launch &
sleep 2

# 启动rtk/GPS
# roslaunch nmea_navsat_driver /opt/ros/noetic/share/nmea_navsat_driver/launch/nmea_serial_driver.launch port:=/dev/ttyUSB2 &
# 这个launch文件在:/opt/ros/noetic/share/nmea_navsat_driver/launch/nmea_serial_driver.launch

roslaunch nmea_navsat_driver nmea_serial_driver.launch &
sleep 2


echo "正在启动rviz..."
rosrun rviz rviz -d sensor.rviz &

echo "系统已启动完成"

# 等待Ctrl+C信号
wait