#pragma once

// #include <Python.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <hdbscan/hdbscan.hpp>
#include <cmath>

namespace rdf
{   
class RobustDynamicFilter
{
public:
    RobustDynamicFilter();
    size_t setLength(size_t length);
    size_t length();
    bool append(pcl::PointCloud<pcl::PointXYZI>::Ptr pcPtr);
    void prt();
    bool alignment();
    bool cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getFiltered();
private:
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> PointCloudList;
    size_t ListLength;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    Eigen::Matrix4f T_Matrix;
    Hdbscan hdbscan;
    std::vector<uint32_t> ClusterNumList;
    std::vector<Eigen::MatrixXd> ClusterMeanList;
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredPointCloud;
    bool load_hdbscan(const Eigen::MatrixXf& PointMatrix);
    double calculateProbability(const Eigen::Vector3d& sample, const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance);
    double median(const Eigen::VectorXd& data);
    double mad(const Eigen::VectorXd& data);
    bool findOutlier(const Eigen::VectorXd& data);
    std::set<int> filterset;
};
}
