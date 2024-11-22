#ifndef ROSHANDLER_H_
#define ROSHANDLER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <queue>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "System.h"
#include <std_msgs/Float32.h>

// Base ImageGrabber class for stereo
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const bool bRect = false, const bool bClahe = false): 
        mpSLAM(pSLAM), do_rectify(bRect), mbClahe(bClahe) {
        mCurrentPose = Sophus::SE3f();
        if(mbClahe)
            mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        
        // Initialize the tracking frequency publisher
        ros::NodeHandle nh;
        tracking_time_pub = nh.advertise<std_msgs::Float32>("tracking_frequency", 10);
        last_tracking_time = ros::Time::now();
    }

    virtual void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
    bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe;
    Sophus::SE3f mCurrentPose;
    ros::Publisher tracking_time_pub;
    ros::Time last_tracking_time;
};

// IMU Grabber class
class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    std::queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

// Derived class for Stereo-Inertial
class StereoInertialGrabber : public ImageGrabber
{
public:
    StereoInertialGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect = false, const bool bClahe = false): 
        ImageGrabber(pSLAM, bRect, bClahe), mpImuGb(pImuGb) {}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    void SyncWithImu();

    std::queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;
    ImuGrabber *mpImuGb;
};

class ROSPublisher
{
private:
    ros::NodeHandle *mpNH;
    ros::Publisher marker_pub, pose_pub;
    tf::TransformBroadcaster tf_broadcaster;
    ORB_SLAM3::System *mpSystem;
    ORB_SLAM3::Atlas *mpAtlas;
    ImageGrabber *mpImageGrabber;
    Eigen::Matrix4f mT_P_R;
    std::thread visualizer_thread;
    std::thread run_thread;

    void publish_pose(Eigen::Vector3f translation, Sophus::SE3f pose);
    void publish_tf(Eigen::Vector3f translation, Eigen::Quaternionf quaternion);
    void publish_markers();
    void run();
    void visualize();

public:
    ROSPublisher(ros::NodeHandle *nh, ORB_SLAM3::System *system, ImageGrabber *pigb, Eigen::Matrix4f T_P_R);
};

#endif