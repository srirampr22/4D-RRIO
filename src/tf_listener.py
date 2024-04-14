#!/usr/bin/env python3
import rospy
import tf

def listen_to_transform():
    rospy.init_node('tf_listener', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)  # Set the rate to 1Hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
            print("Transform from /map to /base_link: Translation", trans, "Rotation", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Transform not available yet.")
        rate.sleep()

if __name__ == '__main__':
    listen_to_transform()
