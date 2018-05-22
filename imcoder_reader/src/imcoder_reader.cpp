#include "imcoder_reader.h"
#include <ros/console.h>

namespace imcoder_reader
{
IMCoder::IMCoder(ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh)
{
    if(!getROSParams(private_nh))
    {
        ROS_ERROR("ROS parameters could not be loaded");
        ros::shutdown();
        return;
    }
    else
    {
        init(nh);
    }
}

void IMCoder::init(ros::NodeHandle& nh){

    // Initialize ROS publishers
    imu_pub_ = nh.advertise<sensor_msgs::Imu>(imu_topic_name_.c_str(), 1);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>(mag_topic_name_.c_str(), 1);

    // Load the RTIMULib.ini config file
    ROS_DEBUG("Loading IMU Settingsy");
    RTIMUSettings *settings_ = new RTIMUSettings(calibration_file_path_.c_str(),
                                                 calibration_file_name_.c_str());

    RTIMU *imu_ = RTIMU::createIMU(settings_);

    if ((imu_ == NULL) || (imu_->IMUType() == RTIMU_TYPE_NULL))
    {
        ROS_ERROR("IMU not found");
        //ros::shutdown();
    }

    // Initialize the imu object
    ROS_DEBUG("Initializing IMU object");
    imu_->IMUInit();

    // Set the Fusion coefficient
    ROS_DEBUG("Setting the fusion coefficient");
    imu_->setSlerpPower(0.02);

    // Enable the sensors
    ROS_DEBUG("Enabling sensors");
    imu_->setGyroEnable(true);
    imu_->setAccelEnable(true);
    imu_->setCompassEnable(true);
}

bool IMCoder::getROSParams(const ros::NodeHandle& private_nh)
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

    if(!private_nh.getParam("imu_topic_name", imu_topic_name_))
    {
        ROS_WARN("No imu_topic_name provided - default: imu/data");
        imu_topic_name_ = "imu/data";
    }
    ROS_DEBUG("imu_topic_name: %s", imu_topic_name_.c_str());

    if(!private_nh.getParam("mag_topic_name", mag_topic_name_))
    {
        ROS_WARN("No topic_name provided - default: imu/mag");
        mag_topic_name_ = "imu/mag";
    }
    ROS_DEBUG("mag_topic_name: %s", mag_topic_name_.c_str());

    if(!private_nh.getParam("calibration_file_path", calibration_file_path_))
    {
        ROS_ERROR("The calibration_file_path parameter must be set to use a calibration file.");
        return false;
    }
    ROS_DEBUG("calibration_file_path: %s", calibration_file_path_.c_str());

    if(!private_nh.getParam("calibration_file_name", calibration_file_name_))
    {
        ROS_WARN("No calibration_file_name provided - default: RTIMULib.ini");
        calibration_file_name_ = "RTIMULib.ini";
    }
    ROS_DEBUG("calibration_file_name: %s", calibration_file_name_.c_str());

    if(!private_nh.getParam("frame_id", frame_id_))
    {
        ROS_WARN("No frame_id provided - default: imu_link");
        frame_id_ = "imu_link";
    }
    ROS_DEBUG("frame_id: %s", frame_id_.c_str());

    return true;
}

void IMCoder::pubData()
{
    imu_msg_.header.stamp                    = ros::Time::now();
    imu_msg_.header.frame_id                 = frame_id_;
    imu_msg_.orientation.x                   = imu_data_.fusionQPose.x();
    imu_msg_.orientation.y                   = imu_data_.fusionQPose.y();
    imu_msg_.orientation.z                   = imu_data_.fusionQPose.z();
    imu_msg_.orientation.w                   = imu_data_.fusionQPose.scalar();
    imu_msg_.angular_velocity.x              = imu_data_.gyro.x();
    imu_msg_.angular_velocity.y              = imu_data_.gyro.y();
    imu_msg_.angular_velocity.z              = imu_data_.gyro.z();
    imu_msg_.linear_acceleration.x           = imu_data_.accel.x();
    imu_msg_.linear_acceleration.y           = imu_data_.accel.y();
    imu_msg_.linear_acceleration.z           = imu_data_.accel.z();

    imu_msg_.orientation_covariance          = {0, 0, 0,
                                                0, 0, 0,
                                                0, 0, 0};

    imu_msg_.angular_velocity_covariance     = {0, 0, 0,
                                                0, 0, 0,
                                                0, 0, 0};

    imu_msg_.linear_acceleration_covariance  = {0, 0, 0,
                                                0, 0, 0,
                                                0, 0, 0};

    imu_pub_.publish(imu_msg_);

    mag_msg_.header.stamp              = ros::Time::now();
    mag_msg_.header.frame_id           = frame_id_;
    mag_msg_.magnetic_field.x          = imu_data_.compass.x();
    mag_msg_.magnetic_field.y          = imu_data_.compass.y();
    mag_msg_.magnetic_field.z          = imu_data_.compass.z();

    mag_msg_.magnetic_field_covariance = {0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0};

    mag_pub_.publish(mag_msg_);
}

void IMCoder::update()
{
    if (imu_->IMURead())
    {
        imu_data_ = imu_->getIMUData();
        pubData();
    }
}

void IMCoder::spin()
{
    ros::Rate update_rate_(1.0 / (imu_->IMUGetPollInterval() / 1000.0));

    while (ros::ok())
    {
        update();
        update_rate_.sleep();
    }
}
} // namespace imcoder_reader