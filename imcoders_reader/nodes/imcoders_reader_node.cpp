#include "imcoders_reader.h"
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imcoders_reader");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    imcoders_reader::IMCoder imcoderReader(nh, nh_private);

    imcoderReader.spin();

    return EXIT_SUCCESS;
}