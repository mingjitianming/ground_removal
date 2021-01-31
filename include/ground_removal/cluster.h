#ifndef CLUSTER_H
#define CLUSTER_H

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

class Cluster
{
    using Point = pcl::PointXYZI;

public:
    Cluster(ros::NodeHandle &nh);
    ~Cluster();
    void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &points);

private:
    std::string points_topic_;
    std::string boxs_topic_;
    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
    ros::Publisher boxs_pub_;
};


#endif