#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Imu.h>

#include "ROSHandler.h"
#include "KeyFrame.h"
#include "MapPoint.h"

using namespace std;

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        mCurrentPose = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mCurrentPose = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }
}

ROSPublisher::ROSPublisher(ros::NodeHandle *nh, ORB_SLAM3::System *system, ImageGrabber *pigb, Eigen::Matrix4f mT_P_R) : 
    mpNH(nh), mpSystem(system), mpImageGrabber(pigb), mT_P_R(mT_P_R)
{
    pose_pub = nh->advertise<geometry_msgs::PoseStamped>("pose", 10);
    marker_pub = nh->advertise<visualization_msgs::Marker>("slam_points", 10);
    mpAtlas = mpSystem->GetAtlas();

    std::thread visualizer_thread(&ROSPublisher::visualize, this);
    std::thread run_thread(&ROSPublisher::run, this);

}

void ROSPublisher::publish_markers() {
    if (mpSystem->GetTrackingState() <2 || mpSystem->GetTrackingState() > 5){
        return;
    }
    vector<ORB_SLAM3::Map*> AllMaps = mpAtlas->GetAllMaps();
    vector<ORB_SLAM3::KeyFrame*> AllKeyFrames;
    vector<ORB_SLAM3::MapPoint*> AllMapPoints;
    for(auto Map : AllMaps){
        vector<ORB_SLAM3::KeyFrame*> KeyFrames = Map->GetAllKeyFrames();
        vector<ORB_SLAM3::MapPoint*> MapPoints = Map->GetAllMapPoints();
        AllKeyFrames.insert(AllKeyFrames.end(), KeyFrames.begin(), KeyFrames.end());
        AllMapPoints.insert(AllMapPoints.end(), MapPoints.begin(), MapPoints.end());
    }

    Eigen::MatrixXf KeyFramePoses(3, AllKeyFrames.size());
    for (int i=0; i<AllKeyFrames.size(); ++i) {
        if(AllKeyFrames[i]->isBad()){
            continue;
        }
        KeyFramePoses.col(i) = AllKeyFrames[i]->GetCameraCenter();
    }
    Eigen::MatrixXf HKeyFramePoses(4, AllKeyFrames.size());
    HKeyFramePoses.topRows(3) = KeyFramePoses;
    HKeyFramePoses.row(3).setOnes();
    HKeyFramePoses = mT_P_R*HKeyFramePoses;


    Eigen::MatrixXf MapPointPoses(3, AllMapPoints.size());
    for (int i=0; i<AllMapPoints.size(); ++i) {
        if(AllMapPoints[i]->isBad()){
            continue;
        }
        MapPointPoses.col(i) = AllMapPoints[i]->GetWorldPos();
    }
    Eigen::MatrixXf HMapPointPoses(4, AllMapPoints.size());
    HMapPointPoses.topRows(3) = MapPointPoses;
    HMapPointPoses.row(3).setOnes();
    HMapPointPoses = mT_P_R*HMapPointPoses;

    visualization_msgs::Marker KeyFramePoints;
    KeyFramePoints.header.frame_id = "map";
    KeyFramePoints.header.stamp = ros::Time::now();
    KeyFramePoints.ns = "KeyFrames";
    KeyFramePoints.action = visualization_msgs::Marker::ADD;
    KeyFramePoints.pose.orientation.w = 1.0;
    KeyFramePoints.id = 0;
    KeyFramePoints.type = visualization_msgs::Marker::POINTS;
    KeyFramePoints.scale.x = 0.1;
    KeyFramePoints.scale.y = 0.1;
    KeyFramePoints.color.g = 1.0f;
    KeyFramePoints.color.a = 1.0;

    for (size_t i = 0; i < AllKeyFrames.size(); ++i) {
        geometry_msgs::Point p;
        p.x = HKeyFramePoses(0, i);
        p.y = HKeyFramePoses(1, i);
        p.z = HKeyFramePoses(2, i);
        KeyFramePoints.points.push_back(p);
    }

    visualization_msgs::Marker MapPointPoints;
    MapPointPoints.header.frame_id = "map";
    MapPointPoints.header.stamp = ros::Time::now();
    MapPointPoints.ns = "MapPoints";
    MapPointPoints.action = visualization_msgs::Marker::ADD;
    MapPointPoints.pose.orientation.w = 1.0;
    MapPointPoints.id = 0;
    MapPointPoints.type = visualization_msgs::Marker::POINTS;
    MapPointPoints.scale.x = 0.1;
    MapPointPoints.scale.y = 0.1;
    MapPointPoints.color.r = 1.0f;
    MapPointPoints.color.a = 1.0;

    for (size_t i = 0; i < HMapPointPoses.size(); ++i) {
        geometry_msgs::Point p;
        if (HMapPointPoses(2, i) > 0 || HMapPointPoses(2, i) < -0.1) 
            continue;
        p.x = HMapPointPoses(0, i);
        p.y = HMapPointPoses(1, i);
        p.z = 0;
        MapPointPoints.points.push_back(p);
    }
    marker_pub.publish(KeyFramePoints);
    marker_pub.publish(MapPointPoints);
}

void ROSPublisher::publish_tf(Eigen::Vector3f translation, Eigen::Quaternionf quaternion) {

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";  // Parent frame
    transform.child_frame_id = "camera_link"; // Child frame
    transform.transform.translation.x = translation.x();  // Translation along x-axis
    transform.transform.translation.y = translation.y();  // Translation along y-axis
    transform.transform.translation.z = translation.z();  // Translation along z-axis

    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    

    // Publish TF transform
    tf_broadcaster.sendTransform(transform);
}
void ROSPublisher::publish_pose(Eigen::Vector3f translation, Sophus::SE3f pose) {
    Eigen::Matrix3f rotationMatrix = pose.rotationMatrix();
    // Extract the z-axis (3rd column of the rotation matrix)
    Eigen::Vector3f zAxis = rotationMatrix.col(2);

    // Project the z-axis onto the x-y plane by setting the z component to zero
    Eigen::Vector3f zAxisProjectedXY(zAxis.x(), zAxis.y(), 0);
    float rotationAngle = atan2(zAxisProjectedXY.y(), zAxisProjectedXY.x()) + 3.141592;

    // Pose Message (example: at the same position as TF)
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map"; // Frame in which the pose is expressed
    pose_msg.pose.position.x = translation.x();         // Same translation as TF
    pose_msg.pose.position.y = translation.y();         // Same translation as TF
    pose_msg.pose.position.z = translation.z();         // Same translation as TF

    pose_msg.pose.orientation =  tf::createQuaternionMsgFromYaw(rotationAngle);

    // Publish Pose message
    pose_pub.publish(pose_msg);
} 

void ROSPublisher::run() {
    ros::Rate rate(100.0); // Publish rate: 10 Hz
    while (ros::ok()) {
        Sophus::SE3f pose = Sophus::SE3f(mT_P_R*mpImageGrabber->mCurrentPose.inverse().matrix());
        Eigen::Vector3f translation = pose.translation();
        Eigen::Quaternionf quaternion = pose.unit_quaternion();
        publish_tf(translation, quaternion);
        publish_pose(translation, pose);
        rate.sleep();
    }
}

void ROSPublisher::visualize() {
    ros::Rate rate(1); // Publish rate: 10 Hz
    while (ros::ok()) {
        publish_markers();
        rate.sleep();
    }
}