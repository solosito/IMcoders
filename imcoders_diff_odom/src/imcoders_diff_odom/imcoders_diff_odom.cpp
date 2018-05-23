#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include <ros/console.h>

namespace imcoders_diff_odom
{

typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_type;
typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_type;

imcodersDiffOdom::imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
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

void imcodersDiffOdom::publishOdom(nav_msgs::Odometry& odom_msg)
{
    odom_pub_.publish(odom_msg);
}

void imcodersDiffOdom::imcodersCallback(const sensor_msgs::ImuConstPtr& imcoder_left,
                                        const sensor_msgs::ImuConstPtr& imcoder_right)
{
    sensor_msgs::Imu wheel_l = *imcoder_left;
    sensor_msgs::Imu wheel_r = *imcoder_right;

    tf::Quaternion q_l;
    tf::Quaternion q_r;

    tf::quaternionMsgToTF(wheel_l.orientation, q_l);
    tf::quaternionMsgToTF(wheel_r.orientation, q_r);
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    last_time_ = current_time;

    tfScalar yaw_l, pitch_l, roll_l;
    tfScalar yaw_r, pitch_r, roll_r;

    tf::Matrix3x3 mat_l(q_l);
    tf::Matrix3x3 mat_r(q_r);

    mat_l.getEulerYPR(yaw_l, pitch_l, roll_l);
    mat_r.getEulerYPR(yaw_r, pitch_r, roll_r);

    // m * rad / s
    double v_l = wheel_radius_ * (yaw_l - last_yaw_l) / dt;
    double v_r = wheel_radius_ * (yaw_r - last_yaw_r) / dt;

    // m * rad / s
    double v_sum = (v_r + v_l) / 2.0;
    double v_diff = v_r - v_l;

    // rad
    double dtheta = v_diff / wheel_separation_ * dt;

    // m
    double vx = v_sum * cos(dtheta);
    double vy = v_sum * sin(dtheta);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(dtheta);

    // m/s
    double dx = vx*dt;
    double dy = vy*dt;

    // rad/s
    double wtheta = dtheta/dt;

    nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = odom_child_frame_id_;

    odom.pose.pose.position.x += dx;
    odom.pose.pose.position.y += dy;
    odom.pose.pose.position.z += 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wtheta;

    // TODO: publish tf

    publishOdom(odom);
}

void imcodersDiffOdom::run()
{
    while (ros::ok())
    {
        ros::spin();
    }
}
}  // namespace imcoder_reader