#include "cluster.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle nh("~");

    Cluster cl(nh);

    ros::spin();
    return 0;
}