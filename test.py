import rospy
from sensor_msgs.msg import Imu

def imu_callback(data):
    print("Received IMU data:", data.header.seq)

rospy.init_node('imu_listener')
rospy.Subscriber("/sensor_imu", Imu, imu_callback)
rospy.spin()
