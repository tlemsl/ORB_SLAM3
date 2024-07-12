#ifndef ROSHANDLER_H_
#define ROSHANDLER_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include "System.h"

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM3::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
    Sophus::SE3f mCurrentPose;
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

    void publish_pose(Eigen::Vector3f translation, Sophus::SE3f pose);
    void publish_tf(Eigen::Vector3f translation, Eigen::Quaternionf quaternion);
    void publish_markers();
    void run();
    void visualize();

public:
    ROSPublisher(ros::NodeHandle *nh, ORB_SLAM3::System *system, ImageGrabber *pigb, Eigen::Matrix4f T_P_R);
};

#endif