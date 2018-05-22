#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include <ros/console.h>

namespace imcoders_diff_odom
{

imcodersDiffOdom::imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh)/*,
      imcoder_left_sub_(nh, imcoder_left_topic_name_.c_str(), 1),
      imcoder_right_sub_(nh, imcoder_right_topic_name_.c_str(), 1)*/
{
	if(!init(nh))
    {
        ROS_ERROR("Error during initialization");
        ros::shutdown();
        return;
    };

    getParams(private_nh);
}

bool imcodersDiffOdom::init(ros::NodeHandle& nh)
{
	odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_name_.c_str(), 1);

	message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_(nh, imcoder_left_topic_name_.c_str(), 1);
	message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_(nh, imcoder_right_topic_name_.c_str(), 1);

	typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imcoder_left_sub_, imcoder_right_sub_);
  	sync.registerCallback(boost::bind(&imcodersCallback, _1, _2));


	return true;
}

bool imcodersDiffOdom::getParams(const ros::NodeHandle& private_nh)
{
	if(!private_nh.getParam("odom_topic_name", odom_topic_name_))
    {
        ROS_WARN("No odom_topic_name provided - default: imcoders/diff_odom");
        odom_topic_name_ = "imcoders/diff_odom";
    }

    if(!private_nh.getParam("imcoder_left_topic_name", imcoder_left_topic_name_))
    {
        ROS_WARN("No odom_topic_name provided - default: imcoder_left/Imu");
        imcoder_left_topic_name_ = "imcoder_left/Imu";
    }

    if(!private_nh.getParam("imcoder_right_topic_name", imcoder_right_topic_name_))
    {
        ROS_WARN("No odom_topic_name provided - default: imcoder_right/Imu");
        imcoder_right_topic_name_ = "imcoder_right/Imu";
    }
    return true;
}

void imcodersDiffOdom::publishOdom()
{

}

void imcodersCallback(const sensor_msgs::ImuConstPtr& imcoder_left, const sensor_msgs::ImuConstPtr& imcoder_right)
{
	sensor_msgs::Imu current_imcoder_left = *imcoder_left;
	sensor_msgs::Imu current_imcoder_right = *imcoder_right;
}


void imcodersDiffOdom::run()
{
	while (ros::ok())
	{
		ros::spinOnce();
	}
}
} // namespace imcoder_reader