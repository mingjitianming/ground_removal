#include "cluster.h"

Cluster::Cluster(ros::NodeHandle &nh) : nh_(nh), points_topic_("/ground_less_points"), boxs_topic_("/bounding_boxs")
{
    nh_.param("points_topic", points_topic_, points_topic_);
    nh_.param("boxs_topic", boxs_topic_, boxs_topic_);
    points_sub_ = nh.subscribe(points_topic_, 10, &Cluster::pointsCallback, this);
    boxs_pub_ = nh.advertise<visualization_msgs::MarkerArray>(boxs_topic_, 10, true);
}

Cluster::~Cluster()
{
}

void Cluster::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &points)
{
    pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(*points, *cloud_in);

    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setInputCloud(cloud_in);
    ec.setClusterTolerance(0.3);
    ec.setMaxClusterSize(25000);
    ec.setMinClusterSize(100);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    visualization_msgs::MarkerArray markers;
    markers.markers.clear();

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = points->header.frame_id;
    bbox_marker.header.stamp = points->header.stamp;
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.3;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;

    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_in, cluster_indices[i], min_pt, max_pt);

        bbox_marker.id = marker_id;
        bbox_marker.pose.position.x = (max_pt.x() + min_pt.x()) / 2;
        bbox_marker.pose.position.y = (max_pt.y() + min_pt.y()) / 2;
        bbox_marker.pose.position.z = (max_pt.z() + min_pt.z()) / 2;
        bbox_marker.scale.x = max_pt.x() - min_pt.x();
        bbox_marker.scale.y = max_pt.y() - min_pt.y();
        bbox_marker.scale.z = max_pt.z() - min_pt.z();
        markers.markers.push_back(bbox_marker);
        ++marker_id;
    }

    static int max_marker_size = 0;
    if (markers.markers.size() > max_marker_size)
    {
        max_marker_size = markers.markers.size();
    }

    for (size_t i = marker_id; i < max_marker_size; ++i)
    {
        bbox_marker.id = i;
        bbox_marker.color.a = 0;
        bbox_marker.pose.position.x = 0;
        bbox_marker.pose.position.y = 0;
        bbox_marker.pose.position.z = 0;
        bbox_marker.scale.x = 0;
        bbox_marker.scale.y = 0;
        bbox_marker.scale.z = 0;
        markers.markers.push_back(bbox_marker);
        ++marker_id;
    }
    boxs_pub_.publish(markers);
}