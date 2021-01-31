#include "ground_removal.h"

GroundRemoval::GroundRemoval(ros::NodeHandle &nh)
    : nh_(nh),
      points_topic_("/rslidar_points"),
      ground_less_topic_("ground_less"),
      ground_topic_("ground"),
      ground_threshold_(-1),
      num_lpr_(20),
      seeds_threshold_(1.2),
      dist_(0.3),
      num_iter_(3),
      filter_ground(new pcl::PointCloud<Point>)
{
    nh_.param("points_topic", points_topic_, points_topic_);
    nh_.param("ground_threshold", ground_threshold_, ground_threshold_);
    nh_.param("num_lpr", num_lpr_, num_lpr_);
    nh_.param("seeds_threshold", seeds_threshold_, seeds_threshold_);
    nh_.param("dist_plane", dist_, dist_);
    nh_.param("num_iter", num_iter_, num_iter_);
    nh_.param("ground_topic", ground_topic_, ground_topic_);
    nh_.param("ground_less_topic", ground_less_topic_, ground_less_topic_);

    points_sub_ = nh_.subscribe(points_topic_, 10, &GroundRemoval::extractGround, this);
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_points", 10, true);
    ground_less_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_less_points", 10, true);
}

void GroundRemoval::extractGround(const sensor_msgs::PointCloud2ConstPtr points)
{
    std::cout << "receive points" << std::endl;
    pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(*points, *cloud_in);
    std::vector<int> id;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, id);
    // std::sort(cloud_in->points.begin(), cloud_in->points.end(), [](const Point &a, const Point &b) { return a.z < b.z; });

    //FIXME: 析构时会保持
    // pcl::PointCloud<Point>::Ptr filter_ground(new pcl::PointCloud<Point>);
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3, ground_threshold_);
    pass.filter(*filter_ground);

    pcl::PointCloud<Point>::Ptr ground = extractSeeds(filter_ground);
    pcl::PointCloud<Point>::Ptr ground_less(new pcl::PointCloud<Point>);
    for (int i = 0; i < num_iter_; ++i)
    {
        Eigen::Vector3f normal = estimatePlane(ground);

        Eigen::MatrixXf points_mat(cloud_in->points.size(), 3);
        for (int i = 0; i < cloud_in->points.size(); ++i)
        {
            points_mat.row(i) << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z;
        }

        Eigen::VectorXf ds = points_mat * normal;

        ground->clear();
        for (int i = 0; i < points_mat.rows(); ++i)
        {
            if (ds[i] < dist_d_)
            {
                ground->points.emplace_back(cloud_in->points[i]);
            }
            else
            {
                ground_less->points.emplace_back(cloud_in->points[i]);
            }
        }
    }
    filter_ground->clear();

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground, ground_msg);
    ground_msg.header = points->header;
    ground_pub_.publish(ground_msg);

    sensor_msgs::PointCloud2 ground_less_msg;
    pcl::toROSMsg(*ground_less, ground_less_msg);
    ground_less_msg.header = points->header;
    ground_less_pub_.publish(ground_less_msg);
}

pcl::PointCloud<Point>::Ptr GroundRemoval::extractSeeds(const pcl::PointCloud<Point>::Ptr &cloud)
{
    float sum_z = 0;
    int i = 0;
    for (; i < cloud->points.size() && i < num_lpr_; ++i)
    {
        sum_z += cloud->points[i].z;
    }

    double lpr_height = i != 0 ? sum_z / i : 0;

    pcl::PointCloud<Point>::Ptr seeds(new pcl::PointCloud<Point>);

    for (auto &point : cloud->points)
    {
        if (point.z < lpr_height + seeds_threshold_)
            seeds->points.emplace_back(point);
    }
    return seeds;
}

Eigen::Vector3f GroundRemoval::estimatePlane(pcl::PointCloud<Point>::Ptr &ground_candidate)
{
    Eigen::Vector4f centroid;
    Eigen::Matrix3f cov;
    pcl::computeMeanAndCovarianceMatrix(*ground_candidate, cov, centroid);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    Eigen::Vector3f normal = svd.matrixU().col(2);
    dist_d_ = dist_ + (normal.transpose() * centroid.head<3>())(0, 0);
    return normal;
}