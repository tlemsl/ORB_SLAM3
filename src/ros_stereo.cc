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
#include "ROSHandler.h"
#include "MapConverter.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SLAM");
    ros::start();

    // Node handle to interact with the parameter server
    ros::NodeHandle nh("~");

    std::string vocabulary_path;
    std::string settings_path;
    std::string file_path;

    bool visualize = true;
    

    // Retrieve parameters from the parameter server
    // if (!nh.getParam("vocabulary_path", vocabulary_path) ||
    //     !nh.getParam("settings_path", settings_path) ||
    //     !nh.getParam("visualize", visualize))
    if(argc != 4)
    {
        ROS_ERROR("Failed to get parameters. Usage: rosrun ORB_SLAM3 Stereo _vocabulary_path:=path_to_vocabulary _settings_path:=path_to_settings _do_rectify:=true/false");
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualize);
    ImageGrabber igb(&SLAM);
    igb.do_rectify = false;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed2i/zed_node/left_raw/image_raw_gray", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed2i/zed_node/right_raw/image_raw_gray", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    
    if(nh.getParam("file_path", file_path)){
        Eigen::Vector3f normal;
        Eigen::Vector3f point;
        if (loadPlaneData(file_path, normal, point)) {
            std::cout << "Loaded plane normal: " << normal.transpose() << std::endl;
            std::cout << "Loaded plane point: " << point.transpose() << std::endl;
        } else {
            ROS_ERROR("Failed to load plane data.");
            return -1;
        }
        Eigen::Matrix4f T_P_R = getPlaneTransformation(normal, point).inverse();
        ROSPublisher rospub(&nh, &SLAM, &igb, T_P_R);
    }
    else {
        std::cout << "No plane parameter file!" << std::endl;
    }
    // Stop all threads
    ros::spin();
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}