#include <ros/ros.h>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"ground_removal");
    ros::NodeHandle("~");

    ros::spin();
    return 0;
}