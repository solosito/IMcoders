#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include <ros/console.h>

namespace imcoders_diff_odom
{

typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_type;
typedef message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_type;

imcodersDiffOdom::imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      last_time_(0.0),
      last_x_(0.0),
      last_y_(0.0),
      last_theta_(0.0),
      last_q_l_(0.0, 0.0, 0.0, 1.0),
      last_q_r_(0.0, 0.0, 0.0, 1.0),
      last_orientation_imu_(0.0, 0.0, 1.0),
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
    if(private_nh.getParam("debug_mode", debug_mode_))
    {
        if(debug_mode_)
        {
            if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            {
                ros::console::notifyLoggerLevelsChanged();
            }
            ROS_DEBUG("ROS console set on DEBUG mode");            
        }
    }

    if(!private_nh.getParam("publish_tf", publish_tf_))
    {
        ROS_WARN("No publish_tf provided - default: true");
        publish_tf_ = true;
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
        // Transform geometry_msgs::Quaternion to tf::Quaternion
        tf::Quaternion q_l;
        tf::Quaternion q_r;

        tf::quaternionMsgToTF(wheel_l.orientation, q_l);
        tf::quaternionMsgToTF(wheel_r.orientation, q_r);

        // Get rotation quaternion between current quaternion and last one
        tf::Quaternion dq_l = (last_q_l_.inverse() * q_l).normalize();
        tf::Quaternion dq_r = (last_q_r_.inverse() * q_r).normalize();

        // Define rotation axis
        tf::Vector3 z_axis(0.0, 0.0, 1.0);

        // Get IMU differential rotation
        tf::Vector3 diff_ori_imu_l = (tf::quatRotate(dq_l, z_axis)).normalize();
        tf::Vector3 diff_ori_imu_r = (tf::quatRotate(dq_r, z_axis)).normalize();

        // Get direction of rotation using dot product
        tf::Vector3 cross_vector_l = z_axis.cross(diff_ori_imu_l);
        tf::Vector3 cross_vector_r = z_axis.cross(diff_ori_imu_r);

        double direction_l = ( cross_vector_l.y() > 0.0) ? 1.0 : -1.0;
        double direction_r = ( cross_vector_r.y() > 0.0) ? 1.0 : -1.0;

        double dot_product_l = z_axis.dot(diff_ori_imu_l);
        double dot_product_r = z_axis.dot(diff_ori_imu_r);

        // Get the signed pitch
        double dpitch_l = acosf(dot_product_l) * direction_l;
        double dpitch_r = acosf(dot_product_r) * direction_r;

        if(std::isnan(dpitch_l))
            dpitch_l = 0.0;
        if(std::isnan(dpitch_r))
            dpitch_r = 0.0;

        cum_pitch_l_ += dpitch_l;

        // Odometry computation
        double w_l = (dpitch_l) / dt;
        double w_r = (dpitch_r) / dt;

        double v_l = wheel_radius_ * w_l;
        double v_r = wheel_radius_ * w_r;


        double v_sum = v_r + v_l;
        double v_diff = v_l - v_r;

        double r_icc, w, theta, dtheta, x_icc, y_icc, x, y, dx, dy, vx, vy;

        // Get distance between ICC and the midpoint of the wheel axis
        if(fabs(v_diff) > dtheta_threshold_)
        {
            r_icc = wheel_separation_ / 2.0 * v_sum / v_diff;

            // Get angular velocity for robot
            w = v_diff / wheel_separation_;

            // Get robot heading
            dtheta = w * dt;
            theta = fmod(dtheta + last_theta_,2.0*M_PI);

            // Get center of rotation
            x_icc = last_x_ - r_icc * sinf (last_theta_);
            y_icc = last_y_ + r_icc * cosf (last_theta_);

            // Get robot new position
            x = cosf(w * dt) * (last_x_ - x_icc) - sinf(w * dt) * (last_y_ - y_icc) + x_icc;
            y = sinf(w * dt) * (last_x_ - x_icc) + cosf(w * dt) * (last_y_ - y_icc) + y_icc;

            // Get travelled distances
            dx = x - last_x_;
            dy = y - last_y_;

            // Get robot linear velocities (w already computed)
            vx = dx/dt;
            vy = dy/dt;
        }
        else
        {
            w = 0.0;
            theta = last_theta_;

            vx = v_sum / 2.0;
            vy = 0.0;

            dx = vx * dt;
            dy = vy * dt;

            x = last_x_ + dx;
            y = last_y_ + dy;
        }

        if(std::isnan(theta))
        {
            ros::shutdown();

        }

        // Transform yaw rotation in euler angles to quaternion
        geometry_msgs::Quaternion theta_q = tf::createQuaternionMsgFromYaw(theta);

        // Fill tf msg and publish it
        if(publish_tf_)
        {
            geometry_msgs::TransformStamped odom_tf;

            odom_tf.header.stamp            = current_time;
            odom_tf.header.frame_id         = odom_frame_id_;
            odom_tf.child_frame_id          = odom_child_frame_id_;
            odom_tf.transform.translation.x = x;
            odom_tf.transform.translation.y = y;
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation      = theta_q;

            odom_broadcaster_.sendTransform(odom_tf);            
        }

        // Fill odometry msg and publish it
        nav_msgs::Odometry odom_msg;

        odom_msg.header.stamp          = current_time;
        odom_msg.header.frame_id       = odom_frame_id_;
        odom_msg.child_frame_id        = odom_child_frame_id_;
        odom_msg.pose.pose.position.x  = x;
        odom_msg.pose.pose.position.y  = y;
        odom_msg.pose.pose.position.z  = 0.0;
        odom_msg.pose.pose.orientation = theta_q;
        odom_msg.twist.twist.linear.x  = vx;
        odom_msg.twist.twist.linear.y  = vy;
        odom_msg.twist.twist.angular.z = w;

        odom_pub_.publish(odom_msg);

        // Update values
        last_x_     = x;
        last_y_     = y;
        last_theta_ = theta;
        last_q_l_   = q_l;
        last_q_r_   = q_r;

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
