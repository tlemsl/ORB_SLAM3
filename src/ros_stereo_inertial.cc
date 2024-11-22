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

#include<ros/ros.h>
#include "System.h"
#include "ROSHandler.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);

    ImuGrabber imugb;
    StereoInertialGrabber igb(&SLAM, &imugb, false, false);
    
    // Initialize ROSPublisher with identity transformation matrix
    Eigen::Matrix4f T_P_R = Eigen::Matrix4f::Identity();
    ROSPublisher publisher(&n, &SLAM, &igb, T_P_R);

    ros::Subscriber sub_imu = n.subscribe("/zed2i/zed_node/imu/data", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img_left = n.subscribe("/zed2i/zed_node/left_raw/image_raw_gray", 100, &StereoInertialGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = n.subscribe("/zed2i/zed_node/right_raw/image_raw_gray", 100, &StereoInertialGrabber::GrabImageRight, &igb);

    std::thread sync_thread(&StereoInertialGrabber::SyncWithImu, &igb);

    ros::spin();

    return 0;
}


