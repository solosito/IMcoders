#ifndef IMCODERS_DIFF_ODOM
#define IMCODERS_DIFF_ODOM

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace imcoders_diff_odom
{

class imcodersDiffOdom
{
  public:

	imcodersDiffOdom(ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

	~imcodersDiffOdom() = default;

	///
	/// @brief      Inits the ROS variables (publishers, subscribes and params)
	///
	/// @param      nh    Node for pub/sub
	///
	/// @return     True if right. False otherwise
	///
    bool init(ros::NodeHandle& nh);

    ///
    /// @brief      Gets the parameters.
    ///
	/// @param[in]  private_nh  Private node for ROS parameters
    ///
    /// @return     The parameters.
    ///
    bool getParams(const ros::NodeHandle& private_nh);

    ///
    /// @brief      { function_description }
    ///
	void publishOdom();

	///
	/// @brief      Subscribes to the left and right imcoders sensors in order to compute an odometry
	///
	/// @param[in]  imcoder_left   The left imcoder sensor
	/// @param[in]  imcoder_right  The right imcoder sensor
	///
	void imcodersCallback(const sensor_msgs::ImuConstPtr& imcoder_left, const sensor_msgs::ImuConstPtr& imcoder_right);

	///
	/// @brief      Runs the node
	///
	void run();

private:

	ros::NodeHandle& nh_; // NodeHandle for class, defined outside

	std::string odom_topic_name_;
	std::string imcoder_left_topic_name_;
	std::string imcoder_right_topic_name_;

	sensor_msgs::Imu imcoder_left_msg_;
	sensor_msgs::Imu imcoder_right_msg_;
	nav_msgs::Odometry odom_msg_;

	ros::Publisher odom_pub_;
	/*message_filters::Subscriber<sensor_msgs::Imu> imcoder_left_sub_(nh, imcoder_left_topic_name_.c_str(), 1);*/
	/*message_filters::Subscriber<sensor_msgs::Imu> imcoder_right_sub_(nh, imcoder_right_topic_name_.c_str(), 1);*/
	//message_filters::Synchronizer<sensor_msgs::Imu, sensor_msgs::Imu> sync_;

};
} // namespace imcoder_reader
#endif // IMCODERS_DIFF_ODOM