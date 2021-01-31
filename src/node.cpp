#include "ground_removal.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_removal");
    ros::NodeHandle nh("~");

    GroundRemoval gr(nh);
    ros::spin();
    return 0;
}