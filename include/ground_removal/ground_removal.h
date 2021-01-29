#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class GroundRemoval
{

public:
    GroundRemoval(ros::NodeHandle &nh);
    ~GroundRemoval();

private:
    void extractGround(const sensor_msgs::PointCloud2ConstPtr points);

private:
    std::string points_topic_;

    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
};

GroundRemoval::GroundRemoval(ros::NodeHandle &nh) : nh_(nh),points_topic_("points_topic")
{
    nh_.param("points_topic", points_topic_, points_topic_);

    points_sub_ = nh_.subscribe(points_topic_, 10, &GroundRemoval::extractGround, this);
}

GroundRemoval::~GroundRemoval()
{
}

void GroundRemoval::extractGround(const sensor_msgs::PointCloud2ConstPtr points)
{
    // pcl::
}

#endif