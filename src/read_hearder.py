#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import pcl
from sensor_msgs.msg import PointCloud
import sensor_msgs.point_cloud2 as pc2

ros_topic = "/radar_enhanced_pcl"

def pcl_callback(data):
    if data is None:
        print("Nothing received")
    else:
        print(ros_topic," pcl frame id:", data.header.frame_id)
    
def header_listener():

    rospy.init_node('header_listener', anonymous=True)
    print("Listening to radar pcl header")

    rospy.Subscriber(ros_topic, PointCloud, pcl_callback)
    rospy.Subscriber(ros_topic, PointCloud, pcl_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    header_listener()