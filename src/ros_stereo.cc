/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <thread>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


#include<opencv2/core/core.hpp>

#include "System.h"

using namespace std;
void slam_visualizer(ORB_SLAM3::System &system);
// Function to get a homogeneous transformation matrix from plane normal and center
Eigen::Matrix4f getPlaneTransformation(const Eigen::Vector3f& normal, const Eigen::Vector3f& center) {
    // Ensure the normal vector is normalized
    Eigen::Vector3f n = normal.normalized();
    
    // Create a basis for the plane
    Eigen::Vector3f u = n.unitOrthogonal().normalized();
    Eigen::Vector3f v = n.cross(u).normalized();
    
    // Construct the rotation matrix
    Eigen::Matrix3f R;
    R.col(0) = u;
    R.col(1) = v;
    R.col(2) = n;
    
    // Construct the translation vector
    Eigen::Vector3f t = center;
    
    // Construct the homogeneous transformation matrix
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    
    return T;
}

Eigen::Vector3f normal(0.00846484,  -0.943085,   0.332444);
Eigen::Vector3f center(2.36297, -0.828457,  -2.47375);
Eigen::Matrix4f T_P_R = getPlaneTransformation(normal, center).inverse();
ros::Publisher marker_pub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    Sophus::SE3f mCurrentPose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SLAM");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // TF Broadcaster
    tf::TransformBroadcaster tf_broadcaster;

    // Pose Publisher
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("slam_points", 10);
    std::thread visualizer_thread(slam_visualizer, std::ref(SLAM));

    ros::Rate rate(100.0); // Publish rate: 10 Hz
    while (ros::ok()) {
        // // Get current time
        // if(SLAM.isFinished())
        //     break;
        // if (SLAM.isLost())
        //     continue;
        
        ros::Time current_time = ros::Time::now();
        Sophus::SE3f pose = Sophus::SE3f(T_P_R*igb.mCurrentPose.matrix());
        Eigen::Vector3f translation = pose.translation();
        Eigen::Quaternion quaternion = pose.unit_quaternion();

        // TF Transform (example: translation along x-axis)

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
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

        Eigen::Matrix3f rotationMatrix = pose.rotationMatrix();
        // Extract the z-axis (3rd column of the rotation matrix)
        Eigen::Vector3f zAxis = rotationMatrix.col(2);
        
        // Project the z-axis onto the x-y plane by setting the z component to zero
        Eigen::Vector3f zAxisProjectedXY(zAxis.x(), zAxis.y(), 0);
        float rotationAngle = atan2(zAxisProjectedXY.y(), zAxisProjectedXY.x()) + 3.141592;

        // Pose Message (example: at the same position as TF)
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "map"; // Frame in which the pose is expressed
        pose_msg.pose.position.x = translation.x();         // Same translation as TF
        pose_msg.pose.position.y = translation.y();         // Same translation as TF
        pose_msg.pose.position.z = translation.z();         // Same translation as TF

        pose_msg.pose.orientation =  tf::createQuaternionMsgFromYaw(rotationAngle);

        // Publish Pose message
        pose_pub.publish(pose_msg);

        // Sleep to control the publish rate
        rate.sleep();
    }
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");
    spinner.stop();
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mCurrentPose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mCurrentPose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}

void slam_visualizer(ORB_SLAM3::System &system){
    ros::Rate rate(5.0); // Publish rate: 10 Hz
    ORB_SLAM3::Atlas* patlas = system.GetAtlas();
    while (ros::ok()) {
        cout << system.GetTrackingState() << endl;
        if (system.GetTrackingState() <2 || system.GetTrackingState() > 5){
            rate.sleep();
            continue;
        }
        vector<ORB_SLAM3::Map*> AllMaps = patlas->GetAllMaps();
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
        HKeyFramePoses = T_P_R*HKeyFramePoses;


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
        HMapPointPoses = T_P_R*HMapPointPoses;

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


        rate.sleep();
    }
}