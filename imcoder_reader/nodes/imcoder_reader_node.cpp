#include "imcoder_reader.h"
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imcoder_reader");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    imcoder_reader::IMCoder imcoderReader(nh, nh_private);

    imcoderReader.spin();

    return EXIT_SUCCESS;
}