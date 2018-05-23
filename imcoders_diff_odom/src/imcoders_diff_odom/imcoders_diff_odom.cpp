#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include <ros/console.h>

namespace imcoders_diff_odom
{

typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_type;
typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_type;

imcodersDiffOdom::imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      last_time_(0.0),
      last_pitch_l(0.0),
      last_pitch_r(0.0),
      last_theta_(0.0),
      imcoder_left_sub_(NULL),
      imcoder_right_sub_(NULL),
      imcoders_sync_(NULL)
{
    if(getParams(private_nh))
    {
        if (!init(nh))
        {
            ROS_ERROR("Error during initialization");
            ros::shutdown();
            return;
        };
    }
}

bool imcodersDiffOdom::init(ros::NodeHandle& nh)
{
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_name_.c_str(), 1);

    ROS_INFO_STREAM(imcoder_left_topic_name_);
    ROS_INFO_STREAM(imcoder_right_topic_name_);

    int queue_size = 1;

    imcoder_left_sub_ = new imcoder_left_sub_type(nh, imcoder_left_topic_name_.c_str(), queue_size);
    imcoder_right_sub_ = new imcoder_right_sub_type(nh, imcoder_right_topic_name_.c_str(), queue_size);

    imcoders_sync_ = new message_filters::Synchronizer<ImcodersSyncPolicy>(ImcodersSyncPolicy(queue_size), *imcoder_left_sub_, *imcoder_right_sub_);
    imcoders_sync_->registerCallback(boost::bind(&imcodersDiffOdom::imcodersCallback, this, _1, _2));

    return true;
}

bool imcodersDiffOdom::getParams(const ros::NodeHandle& private_nh)
{

    if (!private_nh.getParam("odom_topic_name", odom_topic_name_))
    {
        ROS_WARN("No odom_topic_name provided - default: imcoders/diff_odom");
        odom_topic_name_ = "imcoders/diff_odom";
    }

    if (!private_nh.getParam("odom_frame", odom_frame_id_))
    {
        ROS_WARN("No odom_frame provided - default: imcoders_odom");
        odom_topic_name_ = "imcoders_odom";
    }

    if (!private_nh.getParam("odom_child_frame_id", odom_child_frame_id_))
    {
        ROS_WARN("No odom_child_frame_id provided - default: base_link");
        odom_child_frame_id_ = "base_link";
    }

    if (!private_nh.getParam("imcoder_left_topic_name", imcoder_left_topic_name_))
    {
        ROS_WARN("No imcoder_left_topic_name provided - default: imcoder_left/Imu");
        imcoder_left_topic_name_ = "imcoder_left/Imu";
    }

    if (!private_nh.getParam("imcoder_right_topic_name", imcoder_right_topic_name_))
    {
        ROS_WARN("No imcoder_right_topic_name provided - default: imcoder_right/Imu");
        imcoder_right_topic_name_ = "imcoder_right/Imu";
    }

    if (!private_nh.getParam("wheel_radius", wheel_radius_))
    {
        ROS_ERROR("No wheel_radius provided");
        return false;
    }

    if (!private_nh.getParam("wheel_separation", wheel_separation_))
    {
        ROS_ERROR("No wheel_separation provided");
        return false;
    }

    return true;
}

void imcodersDiffOdom::imcodersCallback(const sensor_msgs::ImuConstPtr& imcoder_left,
                                        const sensor_msgs::ImuConstPtr& imcoder_right)
{
    // Get info from imcoders attached to the wheels
    sensor_msgs::Imu wheel_l = *imcoder_left;
    sensor_msgs::Imu wheel_r = *imcoder_right;

    // Compute time difference
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();

    // Transform geometry_msgs::Quaternion to tf::Quaternion
    tf::Quaternion q_l;
    tf::Quaternion q_r;

    tf::quaternionMsgToTF(wheel_l.orientation, q_l);
    tf::quaternionMsgToTF(wheel_r.orientation, q_r);

    // Transform quaternion to Euler angles
    tfScalar yaw_l, pitch_l, roll_l;
    tfScalar yaw_r, pitch_r, roll_r;

    tf::Matrix3x3 mat_l(q_l);
    tf::Matrix3x3 mat_r(q_r);

    mat_l.getEulerYPR(yaw_l, pitch_l, roll_l);
    mat_r.getEulerYPR(yaw_r, pitch_r, roll_r);

    // Odometry computation

    // Get linear velocity for each wheel ( v = r * dpitch / dt )
    double v_l = wheel_radius_ * (pitch_l - last_pitch_l) / dt;
    double v_r = wheel_radius_ * (pitch_r - last_pitch_r) / dt;

    // Get mean velocity at the center of the robot
    double v_mean  = (v_r + v_l) / 2.0;
    double v_diff = v_r - v_l;

    double dtheta = v_diff / wheel_separation_; // angle speed in radians/sec
    double theta = last_theta_ + dtheta * dt;   // abs. angle in global

    double vx = v_mean * cos(theta);
    double vy = v_mean * sin(theta);

    double dx = vx * dt;
    double dy = vy * dt;

    geometry_msgs::Quaternion odom_q = tf::createQuaternionMsgFromYaw(dtheta * dt);

    // Fill tf msg and publish it
    geometry_msgs::TransformStamped odom_tf;

    odom_tf.header.stamp            = current_time;
    odom_tf.header.frame_id         = odom_frame_id_;
    odom_tf.child_frame_id          = odom_child_frame_id_;
    odom_tf.transform.translation.x = dx;
    odom_tf.transform.translation.y = dy;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation      = odom_q;

    odom_broadcaster_.sendTransform(odom_tf);

    // Fill odometry msg and publish it
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp           = current_time;
    odom_msg.header.frame_id        = odom_frame_id_;
    odom_msg.child_frame_id         = odom_child_frame_id_;
    odom_msg.pose.pose.position.x  += dx;
    odom_msg.pose.pose.position.y  += dy;
    odom_msg.pose.pose.position.z  += 0.0;
    odom_msg.pose.pose.orientation  = odom_q;
    odom_msg.twist.twist.linear.x   = vx;
    odom_msg.twist.twist.linear.y   = vy;
    odom_msg.twist.twist.angular.z  = dtheta;

    odom_pub_.publish(odom_msg);

    // Update values
    last_pitch_l = pitch_l;
    last_pitch_r = pitch_r;
    last_theta_ = theta;
    last_time_ = current_time;
}

void imcodersDiffOdom::run()
{
    while (ros::ok())
    {
        ros::spin();
    }
}
}  // namespace imcoder_reader