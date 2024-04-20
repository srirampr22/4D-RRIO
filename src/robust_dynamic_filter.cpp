#include <iostream>
#include <robust_dynamic_filter.h>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <hdbscan/hdbscan.hpp>
#include <cmath>

using namespace std;
using namespace rdf;

RobustDynamicFilter::RobustDynamicFilter() {
    cout << "RD Filter Start" << endl;
    ListLength = 2; // Set Memory length
    gicp.setMaximumIterations(10); // Set maximum iterations
    gicp.setTransformationEpsilon(1e-8); // Set convergence criteria
    gicp.setMaxCorrespondenceDistance(0.05); // Set maximum correspondence distance
}

void RobustDynamicFilter::prt() {
    cout << "Size: " << PointCloudList.size() << endl;
}

size_t RobustDynamicFilter::setLength(size_t length) {
    ListLength = length;
    return ListLength;
}

size_t RobustDynamicFilter::length() {
    return ListLength;
}

bool RobustDynamicFilter::append(pcl::PointCloud<pcl::PointXYZI>::Ptr pcPtr) {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pcPtr);
    sor.setLeafSize(2.0f, 2.0f, 2.0f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*pcPtr, *filtered_cloud);
    sor.filter(*cloud_downsample);
    // cout << pcPtr->size() << " | " << cloud_downsample->size() << endl;
    pcl::IndicesConstPtr indices = sor.getIndices();

    cluster_segment(cloud_downsample);
    vector<size_t> remove_list;
    for (size_t i = 0; i < filtered_cloud->size(); i++) {
        size_t ind = (*indices)[i];
        if (hdbscan.normalizedLabels_[ind] == -1) {
            remove_list.push_back(i);
        }
    }
    while (!remove_list.empty()) {
        auto i = remove_list.back();
        remove_list.pop_back();
        filtered_cloud->erase(filtered_cloud->begin() + i);
    }

    PointCloudList.push_back(filtered_cloud);
    ClusterNumList.push_back(hdbscan.numClusters_);

    auto ClusterType = hdbscan.numClusters_;
    Eigen::MatrixXd ClusterMean;
    ClusterMean.resize(ClusterType, 4);
    ClusterMean.setZero();
    for (size_t i = 0; i < cloud_downsample->size(); i++) {
        auto label = hdbscan.normalizedLabels_[i];
        if (label != -1) {
            ClusterMean(label-1, 0) = cloud_downsample->points[i].x;
            ClusterMean(label-1, 1) = cloud_downsample->points[i].y;
            ClusterMean(label-1, 2) = cloud_downsample->points[i].z;
            ClusterMean(label-1, 3) += 1;
        }
    }
    ClusterMean = ClusterMean.array().colwise() / ClusterMean.col(3).array();
    ClusterMeanList.push_back(ClusterMean);

    // cout << pcPtr->size() << " | " << filtered_cloud->size() << " | " << endl;

    if (PointCloudList.size() > ListLength) {
        PointCloudList.erase(PointCloudList.begin());
        ClusterNumList.erase(ClusterNumList.begin());
        ClusterMeanList.erase(ClusterMeanList.begin());
    }
    bool result = alignment();

    if (result == false) {
        FilteredPointCloud = filtered_cloud;
        return false;
    }

    Eigen::Matrix3d covariance;
    covariance << 0.03, 0.0, 0.0,
                  0.0, 0.03, 0.0,
                  0.0, 0.0, 0.03;

    if (ClusterNumList.back() == ClusterNumList[0]) {
        Eigen::MatrixXd ClusterDiff = ClusterMeanList.back() - ClusterMeanList[0];
        Eigen::VectorXd ProbList;
        ProbList.resize(ClusterNumList[0]);
        ClusterDiff = ClusterDiff.block(0, 0, ClusterDiff.rows(), 3);
        // cout << "=============" << endl;
        // cout << ClusterDiff << endl;
        // cout << "=============" << endl;
        // cout << T_Matrix.block(0, 3, 3, 1).transpose() << endl;
        // cout << "=============" << endl;
        for (size_t i = 0; i < ClusterNumList[0]; i++) {
            Eigen::Vector3d sample = ClusterDiff.row(i);
            Eigen::Vector3d mean = T_Matrix.block<3, 1>(0, 2).cast<double>();
            ProbList(i) = calculateProbability(sample, mean, covariance);
        }
        bool isoutlier = findOutlier(ProbList);
        if (isoutlier) {
            pcl::copyPointCloud(*pcPtr, *FilteredPointCloud);
            remove_list.clear();
            for (size_t i = 0; i < FilteredPointCloud->size(); i++) {
                size_t ind = (*indices)[i];
                if (hdbscan.normalizedLabels_[ind] == -1 || filterset.find(hdbscan.normalizedLabels_[ind]) != filterset.end()) {
                    remove_list.push_back(i);
                }
            }
            while (!remove_list.empty()) {
                auto i = remove_list.back();
                remove_list.pop_back();
                FilteredPointCloud->erase(FilteredPointCloud->begin() + i);
            }
        } else {
            FilteredPointCloud = filtered_cloud;
        }
        // cout << "=============" << endl;
        // cout << ProbList << endl;
        // cout << isoutlier << endl;
        // cout << "=============" << endl;
    } else {
        FilteredPointCloud = filtered_cloud;
    }
       
    return true;
}

bool RobustDynamicFilter::alignment() {
    // Create local submap
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_PointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr align_PointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto i = 0; i < PointCloudList.size() - 1; i++) {
        *local_PointCloud += *PointCloudList[i];
    }
    // Local GICP
    if (local_PointCloud->size() > 1 && PointCloudList.size() > 1) {
        cout<<"Local GICP Start"<<endl;
        gicp.setInputSource(PointCloudList.back());
        gicp.setInputTarget(local_PointCloud);
        gicp.align(*align_PointCloud);
        T_Matrix = gicp.getFinalTransformation();
        return true;
    }

    cout<<"Local GICP Fail"<<endl;
    return false;
}

bool RobustDynamicFilter::cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloud_ptr) {
    // Transfer Point Cloud to Eigen Matrix
    Eigen::MatrixXf PointMatrix(PointCloud_ptr->size(), 3);
    for (size_t i = 0; i < PointCloud_ptr->size(); ++i) {
        PointMatrix(i, 0) = PointCloud_ptr->points[i].x;
        PointMatrix(i, 1) = PointCloud_ptr->points[i].y;
        PointMatrix(i, 2) = PointCloud_ptr->points[i].z;
    }
    load_hdbscan(PointMatrix);
    hdbscan.execute(5, 10, "Euclidean");
    // hdbscan.displayResult();
    return true;
}

bool RobustDynamicFilter::load_hdbscan(const Eigen::MatrixXf& PointMatrix) {
    // clear the exist data
    hdbscan.dataset.clear();
    // Convert Eigen Matrix to vector and load the data.
    for (size_t i = 0; i < PointMatrix.rows(); i++) {
        vector<double> row_element;
        row_element.clear();
        for (size_t j = 0; j < PointMatrix.cols(); j++) {
            row_element.push_back(PointMatrix(i, j));
        }
        hdbscan.dataset.push_back(row_element);
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RobustDynamicFilter::getPointCloud() {
    return PointCloudList.back();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RobustDynamicFilter::getFiltered() {
    return FilteredPointCloud;
}

double RobustDynamicFilter::calculateProbability(const Eigen::Vector3d& sample, const Eigen::Vector3d& mean, const Eigen::Matrix3d& covariance) {
    double det = covariance.determinant();
    double normalization = 1.0 / (std::pow(2 * M_PI, 3.0 / 2.0) * std::sqrt(det));
    Eigen::Vector3d diff = sample - mean;
    double exponent = -0.5 * diff.transpose() * covariance.inverse() * diff;
    return normalization * std::exp(exponent);
}

double RobustDynamicFilter::median(const Eigen::VectorXd& data) {
    Eigen::VectorXd sortedData = data;
    std::sort(sortedData.data(), sortedData.data() + sortedData.size());
    int n = sortedData.size();
    if (n % 2 == 0) {
        return 0.5 * (sortedData(n / 2 - 1) + sortedData(n / 2));
    } else {
        return sortedData(n / 2);
    }
}

double RobustDynamicFilter::mad(const Eigen::VectorXd& data) {
    double medianValue = median(data);
    Eigen::VectorXd deviations = (data.array() - data.mean()).abs();
    return median(deviations);
}

bool RobustDynamicFilter::findOutlier(const Eigen::VectorXd& data) {
    // double medianValue = median(data);
    // double madValue = mad(data);
    // double threshold = 10.0 * madValue;
    double threshold = 1e-10;
    filterset.clear();

    for (int i = 0; i < data.size(); ++i) {
        if (data(i) < threshold) {
            filterset.insert(i);
            return true;
        }
    }
    return false;
}

