#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()

def odom_callback(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)


if __name__ == '__main__':
    rospy.init_node('odom_to_path', anonymous = True)

    i_topic = rospy.get_param('~i_odom_topic', 'odom')
    o_topic = rospy.get_param('~o_path_topic', 'path')

    odom_sub = rospy.Subscriber(i_topic, Odometry, odom_callback)
    path_pub = rospy.Publisher(o_topic, Path, queue_size=10)

    rospy.spin()
