#include <iostream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapConverter.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "plane_visualization");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    std::string file_path;
    if (!nh.getParam("file_path", file_path)) {
        ROS_ERROR("Failed to get param 'file_directory'");
        return -1;
    }
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    ORB_SLAM3::Atlas* patlas = SLAM.GetAtlas();
    vector<ORB_SLAM3::Map*> AllMaps = patlas->GetAllMaps();
    vector<ORB_SLAM3::KeyFrame*> AllKeyFrames;
    vector<ORB_SLAM3::MapPoint*> AllMapPoints;
    for(auto Map : AllMaps){
        vector<ORB_SLAM3::KeyFrame*> KeyFrames = Map->GetAllKeyFrames();
        vector<ORB_SLAM3::MapPoint*> MapPoints = Map->GetAllMapPoints();
        AllKeyFrames.insert(AllKeyFrames.end(), KeyFrames.begin(), KeyFrames.end());
        AllMapPoints.insert(AllMapPoints.end(), MapPoints.begin(), MapPoints.end());
    }
    // Store the poses in an Eigen matrix
    Eigen::MatrixXf poses(3, AllKeyFrames.size());
    int i = 0;
    cout << AllKeyFrames.size() <<endl;
    for (int i=0; i<AllKeyFrames.size(); ++i) {
        poses.col(i) = AllKeyFrames[i]->GetCameraCenter();
    }

    // Calculate the mean of the points
    Eigen::Vector3f mean = poses.rowwise().mean();
    // Center the matrix by subtracting the mean
    Eigen::MatrixXf centered = poses.colwise() - mean;

    // Calculate the covariance matrix
    Eigen::Matrix3f cov = centered * centered.transpose();

    // Perform eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    
    // The normal to the plane is the eigenvector corresponding to the smallest eigenvalue
    Eigen::Vector3f normal = eig.eigenvectors().col(0);

    cout << "Plane normal: " << normal.transpose() << endl;
    cout << "Plane point: " << mean.transpose() << endl;
    savePlaneData(normal, mean, file_path);
    // Eigen::Matrix4f T_P_R = getPlaneTransformation(normal, mean).inverse();

    // // Publish poses as points

    // Eigen::MatrixXf hpoints(4, AllKeyFrames.size());
    // hpoints.topRows(3) = poses;
    // hpoints.row(3).setOnes();
    // hpoints = T_P_R * hpoints;
    // visualization_msgs::Marker points;
    // points.header.frame_id = "map";
    // points.header.stamp = ros::Time::now();
    // points.ns = "points";
    // points.action = visualization_msgs::Marker::ADD;
    // points.pose.orientation.w = 1.0;
    // points.id = 0;
    // points.type = visualization_msgs::Marker::POINTS;
    // points.scale.x = 0.1;
    // points.scale.y = 0.1;
    // points.color.g = 1.0f;
    // points.color.a = 1.0;

    // for (size_t i = 0; i < AllKeyFrames.size(); ++i) {
    //     geometry_msgs::Point p;
    //     p.x = hpoints(0, i);
    //     p.y = hpoints(1, i);
    //     p.z = hpoints(2, i);
    //     points.points.push_back(p);
    // }
    // // Publish plane as a polygon (line strip)
    // visualization_msgs::Marker plane;
    // plane.header.frame_id = "map";
    // plane.header.stamp = ros::Time::now();
    // plane.ns = "plane";
    // plane.action = visualization_msgs::Marker::ADD;
    // plane.pose.orientation.w = 1.0;
    // plane.id = 1;
    // plane.type = visualization_msgs::Marker::LINE_STRIP;
    // plane.scale.x = 0.05; // Thickness of the lines
    // plane.color.r = 1.0f;
    // plane.color.g = 1.0f;
    // plane.color.b = 0.0f;
    // plane.color.a = 0.8;

    // // Generate a few points on the plane
    // Eigen::Vector3f axis1 = eig.eigenvectors().col(1);
    // Eigen::Vector3f axis2 = eig.eigenvectors().col(2);
    // float plane_size = 2.0;
    
    // vector<Eigen::Vector3f> plane_points = {
    //     mean + plane_size * (axis1 + axis2),
    //     mean - plane_size * (axis1 - axis2),
    //     mean - plane_size * (axis1 + axis2),
    //     mean + plane_size * (axis1 - axis2),
    //     mean + plane_size * (axis1 + axis2) // Close the loop
    // };

    // for (const auto& pt : plane_points) {
    //     geometry_msgs::Point p;
    //     p.x = pt.x();
    //     p.y = pt.y();
    //     p.z = pt.z();
    //     plane.points.push_back(p);
    // }

    // while(!ros::isShuttingDown()) {
    //     marker_pub.publish(points);
    //     marker_pub.publish(plane);
    //     sleep(1);
    // }


    // ros::spin();
    return 0;
}