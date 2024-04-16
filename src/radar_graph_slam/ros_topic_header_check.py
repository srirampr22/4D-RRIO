#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud, Imu
import time

def pcl_callback(pcl_msg):
    # print("Received PointCloud message at time", pcl_msg.header.stamp.secs, "seconds and", pcl_msg.header.stamp.nsecs, "nanoseconds.")
    print("Frame ID: " + pcl_msg.header.frame_id)
    print("Child frame ID: ", pcl_msg.header)

def imu_callback(imu_msg):
    # print("Received IMU message at time", imu_msg.header.stamp.secs, "seconds and", imu_msg.header.stamp.nsecs, "nanoseconds.")
    print("Frame ID: " + imu_msg.header.frame_id)
    # print("Child frame ID: " + imu_msg.child_frame_id)

def listener():
    rospy.init_node('header_check_node', anonymous=True)
    rospy.Subscriber("radar_enhanced_pcl", PointCloud, pcl_callback)
    rospy.Subscriber("/vectornav/imu", Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()