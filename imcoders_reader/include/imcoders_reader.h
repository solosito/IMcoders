#ifndef IMCODERS_R_H
#define IMCODERS_R_H

#include <ros/ros.h>
#include <RTIMULib.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

namespace imcoders_reader
{
class IMCoder
{
public:

	IMCoder(ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

	~IMCoder() = default;

	bool getROSParams(const ros::NodeHandle& private_nh);
	void pubData();

	void init(ros::NodeHandle& nh);
	void update();
	void spin();

private:

	ros::NodeHandle& nh_; // NodeHandle for class, defined outside
	sensor_msgs::Imu imu_msg_;
	sensor_msgs::MagneticField mag_msg_;
	std::string imu_topic_name_;
	std::string mag_topic_name_;
	std::string calibration_file_path_;
	std::string calibration_file_name_;
	std::string frame_id_;
	ros::Publisher imu_pub_;
	ros::Publisher mag_pub_;

	RTIMU *imu_;
	RTIMUSettings *settings_;
	RTIMU_DATA imu_data_;

};
} // namespace imcoders_reader

#endif // IMCODERS_R_H