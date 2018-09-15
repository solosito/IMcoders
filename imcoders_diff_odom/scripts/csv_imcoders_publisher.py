#!/usr/bin/env python

import rospy
from numpy import genfromtxt
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion

def PublishFileAtRate(file_01, file_03, pub_rate):

  imu_msg_01 = Imu()
  imu_msg_03 = Imu()

  rate=rospy.Rate(pub_rate)

  for i in range(len(file_01)):
    imu_msg_01.header.stamp = rospy.Time.now()
    imu_msg_03.header.stamp = rospy.Time.now()

    imu_msg_01.header.frame_id = destination_frame_01
    imu_msg_03.header.frame_id = destination_frame_03

    imu_msg_01.orientation.x = file_01[i,1]
    imu_msg_01.orientation.y = file_01[i,2]
    imu_msg_01.orientation.z = file_01[i,3]
    imu_msg_01.orientation.w = file_01[i,4]

    imu_msg_03.orientation.x = file_03[i,1]
    imu_msg_03.orientation.y = file_03[i,2]
    imu_msg_03.orientation.z = file_03[i,3]
    imu_msg_03.orientation.w = file_03[i,4]
    
    imu_msg_publisher_01.publish(imu_msg_01)
    imu_msg_publisher_03.publish(imu_msg_03)

    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('csv_publisher', anonymous = True)

  pub_rate = rospy.get_param('~pub_rate', 100)

  folder = '2018-09-02-18-22-02-complex'
  path_to_file_01 = rospy.get_param('~path_to_file_01', '/home/atroya/imcoders_datasets/'+folder+'/'+folder+'-imcoder01-imu-interpolated.csv')
  path_to_file_03 = rospy.get_param('~path_to_file_03', '/home/atroya/imcoders_datasets/'+folder+'/'+folder+'-imcoder03-imu-interpolated.csv')

  o_topic_01 = rospy.get_param('~o_topic_01', 'imcoder_interp01/imu')
  o_topic_03 = rospy.get_param('~o_topic_03', 'imcoder_interp03/imu')

  destination_frame_01 = rospy.get_param('~destination_frame_01', '/imcoder01_link')
  destination_frame_03 = rospy.get_param('~destination_frame_03', '/imcoder03_link')


  imu_msg_publisher_01 = rospy.Publisher(o_topic_01, Imu, queue_size=1)
  imu_msg_publisher_03 = rospy.Publisher(o_topic_03, Imu, queue_size=1)

  rospy.loginfo("Node %s started", rospy.get_caller_id())
  rospy.loginfo("Reading from files: \n\t%s \n\t%s", path_to_file_01, path_to_file_03)
  rospy.loginfo("Publishing at rate %s into topics: \n\t%s \n\t%s ",pub_rate, o_topic_01, o_topic_03)

  file_01 = genfromtxt(path_to_file_01, delimiter=',')
  file_03 = genfromtxt(path_to_file_03, delimiter=',')

  #while not rospy.is_shutdown():
  PublishFileAtRate(file_01, file_03, pub_rate)