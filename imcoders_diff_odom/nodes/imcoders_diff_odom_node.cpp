#include "imcoders_diff_odom/imcoders_diff_odom.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imcoders_diff_odom");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    imcoders_diff_odom::imcodersDiffOdom imcodersDiffOdom(nh, nh_private);

    imcodersDiffOdom.run();

    return EXIT_SUCCESS;
}