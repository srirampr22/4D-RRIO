#include "utility_radar.h"

#include <tf/transform_broadcaster.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subRadarOdometry;

    ros::Publisher pubOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f radarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tflistener;
    tf::StampedTransform radar2radar_link;

    double radarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    {
        subImuOdometry = nh.subscribe<nav_msgs::Odometry>("/imu_odom_incremental", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subRadarOdometry = nh.subscribe<nav_msgs::Odometry>("/radar_odom", 15, &TransformFusion::radarOdometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubOdometry = nh.advertise<nav_msgs::Odometry>("/fusion_odom", 2000);
        pubImuPath = nh.advertise<nav_msgs::Path>("/fusion_path", 2000);

        // radarOdomAffine = Eigen::Affine3d::Identity();
        // imuOdomAffineFront = Eigen::Affine3d::Identity();
        // imuOdomAffineBack = Eigen::Affine3d::Identity();
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void radarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (odomMsg == nullptr) {
            ROS_INFO("radarOdomMsg is empty");
        }

        radarOdomAffine = odom2affine(*odomMsg);

        radarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        if (odomMsg == nullptr) {
            ROS_INFO("imuOdomIncreMsg is empty");
        }
            
        // static tf::TransformBroadcaster tfWorld2Odom;
        // static tf::Transform world_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        // tfWorld2Odom.sendTransform(tf::StampedTransform(world_to_odom, odomMsg->header.stamp, "world", "odom"));

        imuOdomQueue.push_back(*odomMsg);

        if (radarOdomTime < 0)
            return;
        
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() < radarOdomTime - 0.1)
                imuOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = radarOdomAffine * imuOdomAffineIncre; // check this

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

        // publish latest odometry
        nav_msgs::Odometry radarOdometry = imuOdomQueue.back();
        // cout<<"Radar Odometry: "<<radarOdometry<<endl;
        radarOdometry.pose.pose.position.x = x;
        radarOdometry.pose.pose.position.y = y;
        radarOdometry.pose.pose.position.z = z;
        radarOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubOdometry.publish(radarOdometry);
        // cout<<"Succesfully Published Radar Odometry: "<<radarOdometry<<endl;

        static tf::TransformBroadcaster tfodom2radarlink;
        tf::Transform tcurr;
        tf::poseMsgToTF(radarOdometry.pose.pose, tcurr);
        tfodom2radarlink.sendTransform(tf::StampedTransform(tcurr, radarOdometry.header.stamp, "odom", "car_link"));

        // publish path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1) {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            this_pose_stamped.header.frame_id = "car_link";
            this_pose_stamped.pose = radarOdometry.pose.pose;
            imuPath.poses.push_back(this_pose_stamped);
            // while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < radarOdomTime - 1.0)
            //     imuPath.poses.erase(imuPath.poses.begin());
            // if (pubImuPath.getNumSubscribers() != 0)
            // {
            imuPath.header.stamp = imuOdomQueue.back().header.stamp;
            imuPath.header.frame_id = odometryFrame;
            pubImuPath.publish(imuPath);
                
            // }

        }


    }

};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // T_ri: radar frame to imu frame
    // 0.999735807578		-0.0215215701795	-0.0081643477385	-0.3176955976234
    // -0.02148120581797	-0.9997581134183	0.00502853428037	-0.13761019052125
    // -0.00826995351904	-0.0048509797951	-0.99995400578406	0.05898352725152
    // 0			0			0			1
    // T_rl: radar to lidar frame
    //     Radar_to_livox = [0.9987420694356727, -0.02154593184251807, 0.04527979957349116, 0.2960016945940701;
    //  0.02057017793626469, 0.9995486686923027, 0.02190458926318335, -0.1298868374575176;
    //  -0.04573127973037173, -0.02094561026271845, 0.9987339684250071, -0.004543126256660535;
    //  0, 0, 0, 1]
    // T_li: lidar to imu frame
    // lidar to imu :  0.998572 -0.00112588  -0.0534225   -0.613664
    // 0.000314157   -0.999638   0.0269451   -0.267421
    // -0.0534327  -0.0269225   -0.998209   0.0667678
    //         0           0           0           1

    double qx=-0.0269256, qy=0.0000102, qz=0.0003139, qw=0.9996374; 
    double x = -0.613664, y = -0.267421, z = 0.0667678;

    // Eigen::Matrix4d T_ri = (Eigen::Matrix4d() << 
    // 0.999735807578, -0.0215215701795, -0.0081643477385, -0.3176955976234,
    // -0.02148120581797, -0.9997581134183, 0.00502853428037, -0.13761019052125,
    // -0.00826995351904, -0.0048509797951, -0.99995400578406, 0.05898352725152,
    // 0, 0, 0, 1).finished();

    // // Initialize T_rl (radar to lidar frame)
    // Eigen::Matrix4d T_rl = (Eigen::Matrix4d() << 
    //     0.9987420694356727, -0.02154593184251807, 0.04527979957349116, 0.2960016945940701,
    //     0.02057017793626469, 0.9995486686923027, 0.02190458926318335, -0.1298868374575176,
    //     -0.04573127973037173, -0.02094561026271845, 0.9987339684250071, -0.004543126256660535,
    //     0, 0, 0, 1).finished();

    // Eigen::Matrix4d T_li = T_ri * T_rl.inverse();

    // //roataion in quaternion
    // double qx = 0.9998768, qy = -0.0143339, qz = -0.005478, qw = -0.0032931;
    // // translation
    // double x = -0.3176955976234, y = -0.13761019052125, z = 0.05898352725152;

    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(qw, qx, qy, qz);

    // T_bl: tramsform points from radar frame to imu frame 
    gtsam::Pose3 radar2imu = gtsam::Pose3(rotation, gtsam::Point3(x, y, z));

    // T_lb: tramsform points from imu frame to lidar frame (Not sure if this is correct)
    gtsam::Pose3 imu2radar = radar2imu.inverse();

    // IMU noise values
    double init_ax_bias = 0.001;  // Example values
    double init_ay_bias = 0.001;
    double init_az_bias = 0.001;
    double init_gx_bias = 0.001;
    double init_gy_bias = 0.001;
    double init_gz_bias = 0.001;


    IMUPreintegration():imuAccNoise(0.0022281160035059417), imuGyrNoise(0.00011667951042710442), imuAccBiasN(0.00011782392708033614), imuGyrBiasN( 2.616129872371749e-06), imuGravity(9.80511)
    {
        subImu = nh.subscribe<sensor_msgs::Imu>  ("/vectornav/imu", 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("/radar_incremental_odom", 30, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> ("/imu_odom_incremental", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << init_ax_bias, init_ay_bias, init_az_bias, init_gx_bias, init_gy_bias, init_gz_bias).finished()); // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }


    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if(odomMsg == nullptr) {
            ROS_INFO("radarOdomIncreMsg is empty");
        }

        // cout<<"lidar to imu"<<T_li<<endl;


        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty()) {
            ROS_INFO("imuQueOpt is empty");
            return;
        }

     
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;

        

        // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        bool degenerate = true;
        gtsam::Pose3 radarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));    

        // 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            // cout<<"odomMsg is recived: "<<odomMsg<<endl;

            // initial pose
            prevPose_ = radarPose.compose(radar2imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            
            // cout<<"Prev Pose: "<<prevPose_<<endl; 

            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            // cout<<"System Initialized and the key is: "<<key<<endl;
            return;
        }    

        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = radarPose.compose(radar2imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            ROS_INFO("failure with optimization");
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        // cout<<"Key: "<<key<<endl;
        doneFirstOpt = true;
    }

    

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (imu_raw == nullptr) {
            ROS_INFO("imurawMsg is emtpy");
        }
        // else {
        //     ROS_INFO("imurawMsg is not empty");
        // }

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false) {
            // cout<<"doneFirstOpt is false"<<endl;
            return;
        }
            
        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
        // cout<<"interval time: "<<dt<<endl;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);
  
        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "imu_link";

        // transform imu pose to radar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 radarPose = imuPose.compose(imu2radar);

        odometry.pose.pose.position.x = radarPose.translation().x();
        odometry.pose.pose.position.y = radarPose.translation().y();
        odometry.pose.pose.position.z = radarPose.translation().z();
        odometry.pose.pose.orientation.x = radarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = radarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = radarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = radarPose.rotation().toQuaternion().w();


        transformStamped.header.stamp = odometry.header.stamp;
        transformStamped.header.frame_id = odometry.header.frame_id;
        transformStamped.child_frame_id = odometry.child_frame_id;
        transformStamped.transform.translation.x = radarPose.translation().x();
        transformStamped.transform.translation.y = radarPose.translation().y();
        transformStamped.transform.translation.z = radarPose.translation().z();
        transformStamped.transform.rotation.x = odometry.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odometry.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odometry.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = odometry.pose.pose.orientation.w;

        // odom2imu_broadcaster.sendTransform(transformStamped);
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);


    }
private:
    tf::TransformBroadcaster odom2imu_broadcaster; // map => odom_frame
    geometry_msgs::TransformStamped transformStamped;

    double imuAccNoise;
    double imuGyrNoise;
    double imuAccBiasN;
    double imuGyrBiasN;
    double imuGravity;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_preintegration_node");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    // cout<<"IMU Preintegration Node Started"<<endl;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}