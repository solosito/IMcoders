#!/usr/bin/env python

import rospy
import tf
import numpy
from tf.transformations import quaternion_multiply, quaternion_conjugate
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Quaternion

def fromQuaternionStampedToNumpy(quaternionStamped):
    array = numpy.ndarray(shape=(4,1))
    array[0] = quaternionStamped.quaternion.x
    array[1] = quaternionStamped.quaternion.y
    array[2] = quaternionStamped.quaternion.z
    array[3] = quaternionStamped.quaternion.w
    return array

def ImuCallback(data):

    input_q = QuaternionStamped()
    input_q.header = data.header
    input_q.header.frame_id = origin_frame
    input_q.quaternion = data.orientation

    if(tf.canTransform(destination_frame, origin_frame, rospy.Time(0))):
        rotation_q = tf.transformQuaternion(destination_frame, input_q)

        rotation_q_array = fromQuaternionStampedToNumpy(rotation_q)
        input_q_array = fromQuaternionStampedToNumpy(input_q)

        rotated_q_array = quaternion_multiply(quaternion_multiply(rotation_q_array, input_q_array),quaternion_conjugate(rotation_q_array))
        rotated_q = Quaternion(*rotated_q_array)

        newImu = Imu()
        newImu.header.frame_id = destination_frame
        newImu.header.stamp = rospy.Time.now()
        newImu.orientation = rotated_q

        imu_msg_publisher.publish(newImu)

if __name__ == '__main__':
    rospy.init_node('imu_msg_transformer', anonymous = True)

    i_topic = rospy.get_param('~i_topic', 'imu/data')
    o_topic = rospy.get_param('~o_topic', 'imu_tfed/data')
    origin_frame = rospy.get_param('~origin_frame', '/origin_frame')
    destination_frame = rospy.get_param('~destination_frame', '/destination_frame')

    rospy.Subscriber(i_topic, Imu, ImuCallback)

    imu_msg_publisher = rospy.Publisher(o_topic, Imu, queue_size=1)

    tf = tf.TransformListener()

    rospy.loginfo("Node %s started", rospy.get_caller_id())
    rospy.loginfo("Subscribing to topic: %s", i_topic)
    rospy.loginfo("Publishing into topic: /%s", o_topic)

    rospy.spin()