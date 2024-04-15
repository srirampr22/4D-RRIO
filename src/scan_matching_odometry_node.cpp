#include <ros/ros.h>
#include <signal.h>
#include <ctime>
#include <chrono>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include <Eigen/Dense>

#include <radar_graph_slam/ros_utils.hpp>
#include <radar_graph_slam/registrations.hpp>
#include <radar_graph_slam/ScanMatchingStatus.h>
#include <radar_graph_slam/keyframe.hpp>
#include <radar_graph_slam/keyframe_updater.hpp>
#include <radar_graph_slam/graph_slam.hpp>
#include <radar_graph_slam/information_matrix_calculator.hpp>

#include "utility_radar.h"


class ScanMatchingOdometryNode {
public:
    typedef pcl::PointXYZI PointT;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistWithCovarianceStamped, sensor_msgs::PointCloud2> ApproxSyncPolicy;

    
    
    ScanMatchingOdometryNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
        initialize(nh, private_nh);
    }
    

    ~ScanMatchingOdometryNode() {
        // Clean up operations if necessary
    }

    void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
        // Copy all initialization code from `onInit` of the Nodelet version here
        // Replace `getNodeHandle`, `getMTNodeHandle`, and `getPrivateNodeHandle` with the provided nh and private_nh

        initialize_params();

        // Setup subscribers
        ego_vel_sub.reset(new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(nh, "/eagle_data/twist", 256));
        points_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/filtered_points", 32));
        sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *ego_vel_sub, *points_sub));
        sync->registerCallback(boost::bind(&ScanMatchingOdometryNode::pointcloud_callback, this, _1, _2));
    }

    // Transfer all other methods from the Nodelet here unchanged

private:

    void initialize_params() {
    auto& pnh = private_nh;
    points_topic = pnh.param<std::string>("points_topic", "/radar_enhanced_pcl");
    use_ego_vel = pnh.param<bool>("use_ego_vel", true);

    // The minimum tranlational distance and rotation angle between keyframes_.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    enable_transform_thresholding = pnh.param<bool>("enable_transform_thresholding", true);
    enable_imu_thresholding = pnh.param<bool>("enable_imu_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);
    max_diff_trans = pnh.param<double>("max_diff_trans", 1.0);
    max_diff_angle = pnh.param<double>("max_diff_angle", 1.0);
    max_egovel_cum = pnh.param<double>("max_egovel_cum", 1.0);

    map_cloud_resolution = pnh.param<double>("map_cloud_resolution", 0.05);
    keyframe_updater.reset(new radar_graph_slam::KeyframeUpdater(pnh));

    enable_scan_to_map = pnh.param<bool>("enable_scan_to_map", true);
    max_submap_frames = pnh.param<int>("max_submap_frames", 5);

    enable_imu_fusion = private_nh.param<bool>("enable_imu_fusion", false);
    imu_debug_out = private_nh.param<bool>("imu_debug_out", false);
    // cout << "enable_imu_fusion = " << enable_imu_fusion << endl;
    imu_fusion_ratio = private_nh.param<double>("imu_fusion_ratio", 0.1);

    // graph_slam.reset(new GraphSLAM(pnh.param<std::string>("g2o_solver_type", "lm_var")));

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }
    registration_s2s = radar_graph_slam::select_registration_method(pnh);
    registration_s2m = radar_graph_slam::select_registration_method(pnh);
  }

  void pointcloud_callback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twistMsg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }
    // check if twistMsg is valid and cloud_msg is valid
    if (!twistMsg || !cloud_msg) {
      cout<<"twistMsg or cloud_msg is invalid"<<endl;
    }

    timeLaserOdometry = cloud_msg->header.stamp.toSec();
    double this_cloud_time = cloud_msg->header.stamp.toSec();
    static double last_cloud_time = this_cloud_time;

    double dt = this_cloud_time - last_cloud_time;
    double egovel_cum_x = twistMsg->twist.twist.linear.x * dt;
    double egovel_cum_y = twistMsg->twist.twist.linear.y * dt;
    double egovel_cum_z = twistMsg->twist.twist.linear.z * dt;

    // If too large, set 0
    if (pow(egovel_cum_x,2)+pow(egovel_cum_y,2)+pow(egovel_cum_z,2) > pow(max_egovel_cum, 2)) {
      cout << "Too large egovel_cum: " << sqrt(pow(egovel_cum_x,2)+pow(egovel_cum_y,2)+pow(egovel_cum_z,2)) << endl; //This could be an issue
    }
    // else egovel_cum.block<3, 1>(0, 3) = Eigen::Vector3d(egovel_cum_x, egovel_cum_y, egovel_cum_z);
    else {
      egovel_cum.block<3, 1>(0, 3) = Eigen::Vector3d(egovel_cum_x, egovel_cum_y, egovel_cum_z);
      // cout<<"egovel_cum: "<<egovel_cum<<endl;
    }
    
    last_cloud_time = this_cloud_time;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Matching
    Eigen::Matrix4d pose = matching(cloud_msg->header.stamp, cloud);
    geometry_msgs::TwistWithCovariance twist = twistMsg->twist;

    cout<<"pose: "<<pose<<endl;
    // publish map to odom frame
    publish_odometry(cloud_msg->header.stamp, mapFrame, odometryFrame, pose, twist);

    // In offline estimation, point clouds will be supplied until the published time
    // std_msgs::HeaderPtr read_until(new std_msgs::Header());
    // read_until->frame_id = points_topic;
    // read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    // read_until_pub.publish(read_until);

    // read_until->frame_id = "/filtered_points";
    // read_until_pub.publish(read_until);
  }

  void publish_odometry(const ros::Time& stamp, const std::string& father_frame_id, const std::string& child_frame_id, const Eigen::Matrix4d& pose_in, const geometry_msgs::TwistWithCovariance twist_in) {
    // publish transform stamped for IMU integration
    geometry_msgs::TransformStamped odom_trans = radar_graph_slam::matrix2transform(stamp, pose_in, father_frame_id, child_frame_id); //"map" 
    trans_pub.publish(odom_trans);

    // broadcast the transform over TF
    map2odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = father_frame_id;   // frame: /odom
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = pose_in(0, 3);
    odom.pose.pose.position.y = pose_in(1, 3);
    odom.pose.pose.position.z = pose_in(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.twist = twist_in;

    odom_pub.publish(odom);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  Eigen::Matrix4d matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe_cloud_s2s) {
      prev_time = ros::Time();
      prev_trans_s2s.setIdentity();
      keyframe_pose_s2s.setIdentity();
      keyframe_stamp = stamp;
      keyframe_cloud_s2s = cloud;//downsample(cloud);
      registration_s2s->setInputTarget(keyframe_cloud_s2s); // Scan-to-scan
      if (enable_scan_to_map){
        prev_trans_s2m.setIdentity();
        keyframe_pose_s2m.setIdentity();
        keyframe_cloud_s2m = cloud;
        registration_s2m->setInputTarget(keyframe_cloud_s2m);
      }
      return Eigen::Matrix4d::Identity();
    }
    // auto filtered = downsample(cloud);
    auto filtered = cloud;
    // Set Source Cloud
    registration_s2s->setInputSource(filtered);
    if (enable_scan_to_map)
      registration_s2m->setInputSource(filtered);

    std::string msf_source; // This is empty
    Eigen::Isometry3d msf_delta = Eigen::Isometry3d::Identity();

    // cout<<"msf source: "<<msf_source<<endl;

    Eigen::Matrix4d dummy = Eigen::Matrix4d::Identity();

    
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    Eigen::Matrix4d odom_s2s_now;
    Eigen::Matrix4d odom_s2m_now;

    // // **********  Matching  **********

    Eigen::Matrix4d guess;
    if (use_ego_vel) {
      guess = prev_trans_s2s * egovel_cum * msf_delta.matrix();
      // cout<<"using ego vel"<<guess<<endl;
    }   
    else {
      cout<<"warning not using ego vel"<<endl;
      guess = prev_trans_s2s * msf_delta.matrix();
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    registration_s2s->align(*aligned, guess.cast<float>()); // uses ego velogity as a prior guess for the point registration process
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
    s2s_matching_time.push_back(time_used);

    // publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    // If not converged, use last transformation
    if(!registration_s2s->hasConverged()) {
      ROS_INFO("scan matching_ has not converged!!");
      // ROS_INFO("ignore this frame(" << stamp << ")");
      if (enable_scan_to_map) return keyframe_pose_s2m * prev_trans_s2m;
      else return keyframe_pose_s2s * prev_trans_s2s;
    }
    else {
      ROS_INFO("scan matching_ has converged!!");
    }

    Eigen::Matrix4d trans_s2s = registration_s2s->getFinalTransformation().cast<double>();
    // cout<<"scan to scan final registration transformation: "<<trans_s2s<<endl;
    odom_s2s_now = keyframe_pose_s2s * trans_s2s;

    Eigen::Matrix4d trans_s2m;
    if (enable_scan_to_map){
      registration_s2m->align(*aligned, guess.cast<float>());
      if(!registration_s2m->hasConverged()) {
        ROS_INFO("scan matching_ has not converged!!");
        return keyframe_pose_s2m * prev_trans_s2m;
      }
      trans_s2m = registration_s2m->getFinalTransformation().cast<double>();
      odom_s2m_now = keyframe_pose_s2m * trans_s2m;
      // cout<<"scan to map final registration transformation: "<<odom_s2m_now<<endl;
    }

    bool thresholded = false;
    if(enable_transform_thresholding) {

      Eigen::Matrix4d radar_delta;
      if(enable_scan_to_map) radar_delta = prev_trans_s2m.inverse() * trans_s2m; // This is the difference between the previous and current scan to map transformations
      else radar_delta = prev_trans_s2s.inverse() * trans_s2s;
      double dx_rd = radar_delta.block<3, 1>(0, 3).norm(); // This is the norm of the translation vector
      // double da_rd = std::acos(Eigen::Quaterniond(radar_delta.block<3, 3>(0, 0)).w())*180/M_PI;
      Eigen::AngleAxisd rotation_vector;
      rotation_vector.fromRotationMatrix(radar_delta.block<3, 3>(0, 0));
      double da_rd = rotation_vector.angle(); // This is the delta angle of rotation
      Eigen::Matrix3d rot_rd = radar_delta.block<3, 3>(0, 0).cast<double>();
      bool too_large_trans = dx_rd > max_acceptable_trans || da_rd > max_acceptable_angle; // This is a boolean value that checks if the translation or rotation is too large
      double da, dx, delta_rot_imu = 0;
      Eigen::Matrix3d matrix_rot; Eigen::Vector3d delta_trans_egovel; 

      if (too_large_trans) {
          cout << "Too large transform!!  " << dx_rd << "[m] " << da_rd << "[degree]"<<
            " Ignore this frame (" << stamp << ")" << endl;
          prev_trans_s2s = trans_s2s;
          thresholded = true;
          if (enable_scan_to_map){
            prev_trans_s2m = trans_s2m;
            odom_s2m_now = keyframe_pose_s2m * prev_trans_s2m * radar_delta;
          }
          else odom_s2s_now = keyframe_pose_s2s * prev_trans_s2s * radar_delta;
        }

      last_radar_delta = radar_delta;

      }

    prev_time = stamp;
    if (!thresholded) {
      prev_trans_s2s = trans_s2s;
      prev_trans_s2m = trans_s2m;
    }
    bool update = keyframe_updater->decide(Eigen::Isometry3d(odom_s2s_now), stamp);
    if (update) ROS_INFO("Keyframe updated!!");
      
    if(keyframe_updater->decide(Eigen::Isometry3d(odom_s2s_now), stamp)) {
      // Loose Coupling the IMU roll & pitch
      // if (enable_imu_fusion){
      //   if(enable_scan_to_map) transformUpdate(odom_s2m_now);
      //   else radar_graph_slam::transformUpdate(odom_s2s_now);
      // }

      keyframe_cloud_s2s = filtered;
      registration_s2s->setInputTarget(keyframe_cloud_s2s);
      keyframe_pose_s2s = odom_s2s_now;
      cout<<"keyframe_pose_s2s: "<<keyframe_pose_s2s<<endl;
      keyframe_stamp = stamp;
      prev_time = stamp;
      prev_trans_s2s.setIdentity();

      double accum_d = keyframe_updater->get_accum_distance();
      radar_graph_slam::KeyFrame::Ptr keyframe(new radar_graph_slam::KeyFrame(keyframe_index, stamp, Eigen::Isometry3d(odom_s2s_now.cast<double>()), accum_d, cloud));
      keyframe_index ++;
      keyframes.push_back(keyframe);

      if (enable_scan_to_map){
        pcl::PointCloud<PointT>::Ptr submap_cloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::ConstPtr submap_cloud_downsampled;
        for(size_t i=std::max(0, (int)keyframes.size()-max_submap_frames); i < keyframes.size()-1; i++){
          Eigen::Matrix4d rel_pose = keyframes.at(i)->odom_scan2scan.matrix().inverse() * keyframes.back()->odom_scan2scan.matrix();
          pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
          pcl::transformPointCloud(*keyframes.at(i)->cloud, *cloud_transformed, rel_pose);
          *submap_cloud += *cloud_transformed;
        }
        submap_cloud_downsampled = downsample(submap_cloud);
        keyframe_cloud_s2m = submap_cloud_downsampled;
        registration_s2m->setInputTarget(keyframe_cloud_s2m);
        
        keyframes.back()->odom_scan2map = Eigen::Isometry3d(odom_s2m_now);
        keyframe_pose_s2m = odom_s2m_now;
        prev_trans_s2m.setIdentity();
      }

    }

    // if (aligned_points_pub.getNumSubscribers() > 0)
    // {
    //   pcl::transformPointCloud (*cloud, *aligned, odom_s2s_now);
    //   aligned->header.frame_id = odometryFrame;
    //   aligned_points_pub.publish(*aligned);
    // }

    if (enable_scan_to_map)
      return odom_s2m_now;
    else
      return odom_s2s_now;
      

    // return dummy;
   
  }


  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  // ros::Subscriber points_sub;
//   ros::Subscriber msf_pose_sub;
//   ros::Subscriber msf_pose_after_update_sub;
//   ros::Subscriber imu_sub;

  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;
  sensor_msgs::Imu last_frame_imu;
  std::string mapFrame = "map";
  std::string odometryFrame = "odom";

  bool enable_imu_fusion;
  bool imu_debug_out;
  Eigen::Matrix3d global_orient_matrix;  // The rotation matrix with initial IMU roll & pitch measurement (yaw = 0)
    double timeLaserOdometry = 0;
    int imuPointerFront;
    int imuPointerLast;
    double imuTime[radar_graph_slam::imuQueLength];
    float imuRoll[radar_graph_slam::imuQueLength];
    float imuPitch[radar_graph_slam::imuQueLength];
    double imu_fusion_ratio;

  std::unique_ptr<message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>> ego_vel_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> points_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  // Submap
  ros::Publisher submap_pub;
  std::unique_ptr<radar_graph_slam::KeyframeUpdater> keyframe_updater;
  std::vector<radar_graph_slam::KeyFrame::Ptr> keyframes;
  size_t keyframe_index = 0;
  double map_cloud_resolution;
  int  max_submap_frames;
  bool enable_scan_to_map;

  // std::unique_ptr<GraphSLAM> graph_slam;
  // std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  
  ros::Publisher odom_pub;
  ros::Publisher trans_pub;
  // ros::Publisher keyframe_trans_pub;
  ros::Publisher aligned_points_pub;
  ros::Publisher status_pub;
  ros::Publisher read_until_pub;
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster map2odom_broadcaster; // map => odom_frame

  std::string points_topic;


  // keyframe_ parameters
  double keyframe_delta_trans;  // minimum distance between keyframes_
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool enable_transform_thresholding;  //
  bool enable_imu_thresholding;
  double max_acceptable_trans;  //
  double max_acceptable_angle;
  double max_diff_trans;
  double max_diff_angle;
  double max_egovel_cum;
  Eigen::Matrix4d last_radar_delta = Eigen::Matrix4d::Identity();

  // odometry calculation
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose;
  geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose_after_update;

  Eigen::Matrix4d egovel_cum = Eigen::Matrix4d::Identity();
  bool use_ego_vel;

  ros::Time prev_time;
  Eigen::Matrix4d prev_trans_s2s;                  // previous relative transform from keyframe_
  Eigen::Matrix4d keyframe_pose_s2s;               // keyframe_ pose
  Eigen::Matrix4d prev_trans_s2m;
  Eigen::Matrix4d keyframe_pose_s2m;               // keyframe_ pose
  ros::Time keyframe_stamp;                    // keyframe_ time
  pcl::PointCloud<PointT>::ConstPtr keyframe_cloud_s2s;  // keyframe_ point cloud
  pcl::PointCloud<PointT>::ConstPtr keyframe_cloud_s2m;  // keyframe_ point cloud

  // Registration
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration_s2s;    // Scan-to-Scan Registration
  pcl::Registration<PointT, PointT>::Ptr registration_s2m;    // Scan-to-Submap Registration

  // Time evaluation
  std::vector<double> s2s_matching_time;
  ros::Subscriber command_sub;
};



void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Exiting...", signum);
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_matching_odometry_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup signal handling
    signal(SIGINT, signalHandler);

    ScanMatchingOdometryNode node(nh, private_nh);
    // radar_graph_slam::ScanMatchingOdometryNode node(nh, private_nh);

    // Run the node's main operations with ROS spinning
    ros::spin();

    return 0;
}
