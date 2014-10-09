#include <ros/ros.h>
#include "approach_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "approach_controller");
    ros::NodeHandle nh;

    ros::NodeHandle nhPriv("~");
    std::string fixed_frame = "/odom";
    nhPriv.param("fixed_frame", fixed_frame, fixed_frame);

    ApproachController ac("approach_action", fixed_frame);

    ros::spin();
}

