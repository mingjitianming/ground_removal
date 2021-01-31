#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_H

#include <pcl/common/centroid.h>
// #include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using Point = pcl::PointXYZI;
class GroundRemoval
{

public:
    GroundRemoval(ros::NodeHandle &nh);
    ~GroundRemoval() = default;

private:
    void extractGround(const sensor_msgs::PointCloud2ConstPtr points);
    pcl::PointCloud<Point>::Ptr extractSeeds(const pcl::PointCloud<Point>::Ptr &cloud);
    Eigen::Vector3f estimatePlane(pcl::PointCloud<Point>::Ptr &ground_candidate);

private:
    std::string points_topic_;
    std::string ground_topic_;
    std::string ground_less_topic_;
    float ground_threshold_;
    float seeds_threshold_;
    float dist_d_;
    float dist_;
    int num_lpr_;
    int num_iter_;

    pcl::PointCloud<Point>::Ptr filter_ground;

    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher ground_less_pub_;
};

#endif