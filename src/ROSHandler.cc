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
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
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

    ros::Time current_time = ros::Time::now();
    double time_diff = (current_time - last_tracking_time).toSec();
    float frequency = 1.0 / time_diff;  // Convert to Hz
    
    std_msgs::Float32 freq_msg;
    freq_msg.data = frequency;
    tracking_time_pub.publish(freq_msg);
    
    last_tracking_time = current_time;
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void StereoInertialGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void StereoInertialGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

void StereoInertialGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while(true)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();
            
            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && !imgRightBuf.empty())
            {
                imgRightBuf.pop();
                if(!imgRightBuf.empty())
                    tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && !imgLeftBuf.empty())
            {
                imgLeftBuf.pop();
                if(!imgLeftBuf.empty())
                    tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if(!imgLeftBuf.empty() && !imgRightBuf.empty())
            {
                // Get images and timestamps
                cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
                
                this->mBufMutexLeft.lock();
                cv_ptrLeft = cv_bridge::toCvShare(imgLeftBuf.front());
                imgLeftBuf.pop();
                this->mBufMutexLeft.unlock();

                this->mBufMutexRight.lock(); 
                cv_ptrRight = cv_bridge::toCvShare(imgRightBuf.front());
                imgRightBuf.pop();
                this->mBufMutexRight.unlock();

                if(mbClahe)
                {
                    mClahe->apply(cv_ptrLeft->image,imLeft);
                    mClahe->apply(cv_ptrRight->image,imRight);
                }
                else
                {
                    imLeft = cv_ptrLeft->image.clone();
                    imRight = cv_ptrRight->image.clone();
                }

                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                mpImuGb->mBufMutex.lock();
                if(!mpImuGb->imuBuf.empty())
                {
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                    {
                        double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                        cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                        cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                        mpImuGb->imuBuf.pop();
                    }
                }
                mpImuGb->mBufMutex.unlock();

                if(do_rectify)
                {
                    cv::Mat imLeftRect, imRightRect;
                    cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
                    cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
                    mCurrentPose = mpSLAM->TrackStereo(imLeftRect,imRightRect,tImLeft,vImuMeas);
                }
                else
                {
                    mCurrentPose = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
                }
                ros::Time current_time = ros::Time::now();
                double time_diff = (current_time - last_tracking_time).toSec();
                float frequency = 1.0 / time_diff;  // Convert to Hz
                std::cout << "Frequency: " << frequency << std::endl;
                std_msgs::Float32 freq_msg;
                freq_msg.data = frequency;
                tracking_time_pub.publish(freq_msg);
                
                last_tracking_time = current_time;
            }
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
}

ROSPublisher::ROSPublisher(ros::NodeHandle *nh, ORB_SLAM3::System *system, ImageGrabber *pigb, Eigen::Matrix4f mT_P_R) : 
    mpNH(nh), mpSystem(system), mpImageGrabber(pigb), mT_P_R(mT_P_R)
{
    pose_pub = nh->advertise<geometry_msgs::PoseStamped>("pose", 10);
    marker_pub = nh->advertise<visualization_msgs::Marker>("slam_points", 10);
    mpAtlas = mpSystem->GetAtlas();

    visualizer_thread = std::thread(&ROSPublisher::visualize, this);
    run_thread = std::thread(&ROSPublisher::run, this);

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
    ros::Rate rate(30.0); // Publish rate: 10 Hz
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
    ros::Rate rate(3); // Publish rate: 10 Hz
    while (ros::ok()) {
        publish_markers();
        rate.sleep();
    }
}