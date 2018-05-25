#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include <ros/console.h>

namespace imcoders_diff_odom
{

typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_type;
typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_type;

imcodersDiffOdom::imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      last_time_(0.0),
      last_pitch_l_(0.0),
      last_pitch_r_(0.0),
      last_x_(0.0),
      last_y_(0.0),
      last_theta_(0.0),
      imcoder_left_sub_(NULL),
      imcoder_right_sub_(NULL),
      imcoders_sync_(NULL)
{
    if(getParams(private_nh))
    {
        if(!init(nh))
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
    bool debug_mode;
    if(private_nh.getParam("debug_mode", debug_mode))
    {
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
        }
        ROS_DEBUG("ROS console set on DEBUG mode");
    }

    if(!private_nh.getParam("odom_topic_name", odom_topic_name_))
    {
        ROS_WARN("No odom_topic_name provided - default: imcoders/diff_odom");
        odom_topic_name_ = "imcoders/diff_odom";
    }

    if(!private_nh.getParam("odom_frame", odom_frame_id_))
    {
        ROS_WARN("No odom_frame provided - default: imcoders_odom");
        odom_topic_name_ = "imcoders_odom";
    }

    if(!private_nh.getParam("odom_child_frame_id", odom_child_frame_id_))
    {
        ROS_WARN("No odom_child_frame_id provided - default: base_link");
        odom_child_frame_id_ = "base_link";
    }

    if(!private_nh.getParam("imcoder_left_topic_name", imcoder_left_topic_name_))
    {
        ROS_WARN("No imcoder_left_topic_name provided - default: imcoder_left/Imu");
        imcoder_left_topic_name_ = "imcoder_left/Imu";
    }

    if(!private_nh.getParam("imcoder_right_topic_name", imcoder_right_topic_name_))
    {
        ROS_WARN("No imcoder_right_topic_name provided - default: imcoder_right/Imu");
        imcoder_right_topic_name_ = "imcoder_right/Imu";
    }

    if(!private_nh.getParam("wheel_radius", wheel_radius_))
    {
        ROS_ERROR("No wheel_radius provided");
        return false;
    }

    if(!private_nh.getParam("wheel_separation", wheel_separation_))
    {
        ROS_ERROR("No wheel_separation provided");
        return false;
    }

    if(!private_nh.getParam("dtheta_threshold", dtheta_threshold_))
    {
        ROS_WARN("No dtheta_threshold provided - default: 1e-7");
        dtheta_threshold_ = 1e-7;
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

    if(dt != 0.0)
    {
        // Get pitch for each wheel
        double pitch_l = 2.0 * acos(wheel_l.orientation.w);
        double pitch_r = 2.0 * acos(wheel_r.orientation.w);

        // Odometry computation

        // Get angular velocity for each wheel ( w = dpitch / dt )
        double w_l = (pitch_l - last_pitch_l_) / dt;
        double w_r = (pitch_r - last_pitch_r_) / dt;

        // Print debug info
        ROS_DEBUG_STREAM("dt: " << dt);
        ROS_DEBUG_STREAM("pitch_r: " << pitch_r);
        ROS_DEBUG_STREAM("pitch_l: " << pitch_l);
        ROS_DEBUG_STREAM("dpitch_r: " << (pitch_r - last_pitch_r_));
        ROS_DEBUG_STREAM("dpitch_l: " << (pitch_l - last_pitch_l_));
        ROS_DEBUG_STREAM("w_l: " << w_l);
        ROS_DEBUG_STREAM("w_r: " << w_r);

        // Get linear velocity for each wheel ( v = r * w )
        double v_l = wheel_radius_ * w_l;
        double v_r = wheel_radius_ * w_r;

        // Get distance between ICC and the midpoint of the wheel axis
        double r_icc = wheel_separation_ / 2.0 * (v_r + v_l) / (v_r - v_l);

        // Get angular velocity for robot
        double w = (v_r - v_l) / wheel_separation_;

        // Print debug info
        ROS_DEBUG_STREAM("w: " << w);

        // Get robot heading
        double theta = w * dt + last_theta_;;
        double dtheta = theta - last_theta_;

        // If the diference it is too small, do not use it
        if(dtheta < dtheta_threshold_)
            theta = 0.0;

        // Print debug info
        ROS_DEBUG_STREAM("dtheta: " << dtheta);
        ROS_DEBUG_STREAM("theta: " << theta);

        // Get center of rotation
        double x_icc = last_x_ - r_icc * sin (last_theta_);
        double y_icc = last_y_ + r_icc * cos (last_theta_);

        // Get robot new position
        double x = cos(w * dt) * (last_x_ - x_icc) - sin(w * dt) * (last_y_ - y_icc) + x_icc;
        double y = sin(w * dt) * (last_x_ - x_icc) + cos(w * dt) * (last_y_ - y_icc) + y_icc;

        // Print debug info
        ROS_DEBUG_STREAM("x: " << x);
        ROS_DEBUG_STREAM("y: " << y << "\n");

        // Get travelled distances
        double dx = x - last_x_;
        double dy = y - last_y_;

        // Get robot linear velocities (w already computed)
        double vx = dx/dt;
        double vy = dy/dt;

        // Transform yaw rotation in euler angles to quaternion
        geometry_msgs::Quaternion dtheta_q = tf::createQuaternionMsgFromYaw(dtheta);

        // Fill tf msg and publish it
        geometry_msgs::TransformStamped odom_tf;

        odom_tf.header.stamp            = current_time;
        odom_tf.header.frame_id         = odom_frame_id_;
        odom_tf.child_frame_id          = odom_child_frame_id_;
        odom_tf.transform.translation.x = dx;
        odom_tf.transform.translation.y = dy;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation      = dtheta_q;

        odom_broadcaster_.sendTransform(odom_tf);

        // Fill odometry msg and publish it
        nav_msgs::Odometry odom_msg;

        odom_msg.header.stamp          = current_time;
        odom_msg.header.frame_id       = odom_frame_id_;
        odom_msg.child_frame_id        = odom_child_frame_id_;
        odom_msg.pose.pose.position.x  = x;
        odom_msg.pose.pose.position.y  = y;
        odom_msg.pose.pose.position.z  = 0.0;
        odom_msg.pose.pose.orientation = dtheta_q;
        odom_msg.twist.twist.linear.x  = vx;
        odom_msg.twist.twist.linear.y  = vy;
        odom_msg.twist.twist.angular.z = w;

        odom_pub_.publish(odom_msg);

        // Update values
        last_pitch_l_ = pitch_l;
        last_pitch_r_ = pitch_r;
        last_x_       = x;
        last_y_       = y;
        last_theta_   = theta;
    }

    // Update time
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
