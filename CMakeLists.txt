cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   cv_bridge
   message_generation
   geometry_msgs
   nav_msgs
   sensor_msgs
   tf
   eigen_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)
find_package(yaml-cpp REQUIRED)


include_directories(
   ${catkin_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
   ${PROJECT_SOURCE_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
   ${EIGEN3_INCLUDE_DIR}
   ${Pangolin_INCLUDE_DIRS}
   ${YAML_CPP_INCLUDE_DIRS}
)

set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
  -lpthread 
  yaml-cpp
)
# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
endif()

add_library(${PROJECT_NAME}
   src/System.cc
   src/Tracking.cc
   src/LocalMapping.cc
   src/LoopClosing.cc
   src/ORBextractor.cc
   src/ORBmatcher.cc
   src/FrameDrawer.cc
   src/Converter.cc
   src/MapPoint.cc
   src/KeyFrame.cc
   src/Atlas.cc
   src/Map.cc
   src/MapDrawer.cc
   src/Optimizer.cc
   src/Frame.cc
   src/KeyFrameDatabase.cc
   src/Sim3Solver.cc
   src/Viewer.cc
   src/ImuTypes.cc
   src/G2oTypes.cc
   src/CameraModels/Pinhole.cpp
   src/CameraModels/KannalaBrandt8.cpp
   src/OptimizableTypes.cpp
   src/MLPnPsolver.cpp
   src/GeometricTools.cc
   src/TwoViewReconstruction.cc
   src/Config.cc
   src/Settings.cc
   include/System.h
   include/Tracking.h
   include/LocalMapping.h
   include/LoopClosing.h
   include/ORBextractor.h
   include/ORBmatcher.h
   include/FrameDrawer.h
   include/Converter.h
   include/MapPoint.h
   include/KeyFrame.h
   include/Atlas.h
   include/Map.h
   include/MapDrawer.h
   include/Optimizer.h
   include/Frame.h
   include/KeyFrameDatabase.h
   include/Sim3Solver.h
   include/Viewer.h
   include/ImuTypes.h
   include/G2oTypes.h
   include/CameraModels/GeometricCamera.h
   include/CameraModels/Pinhole.h
   include/CameraModels/KannalaBrandt8.h
   include/OptimizableTypes.h
   include/MLPnPsolver.h
   include/GeometricTools.h
   include/TwoViewReconstruction.h
   include/SerializationUtils.h
   include/Config.h
   include/Settings.h
)
target_link_libraries(${PROJECT_NAME}
   ${LIBS} 
)

add_library(ROS_lib
   src/ROSHandler.cc
)

target_link_libraries(ROS_lib
   ${PROJECT_NAME}
   ${catkin_LIBRARIES} 
)

add_library(Convert_lib
   src/MapConverter.cc
)

target_link_libraries(
   ${PROJECT_NAME}
)

# add_executable(stereo_inertial
#    src/ros_stereo_inertial.cc
# )
add_executable(stereo
   src/ros_stereo.cc
)
add_executable(stereo_inertial
   src/ros_stereo_inertial.cc
)
add_executable(mono
   src/ros_mono.cc
)
add_executable(map_converter
   src/Convert_node.cc
)

# target_link_libraries(stereo_inertial
#    ${catkin_LIBRARIES} ${PROJECT_NAME}
# )
target_link_libraries(stereo
   ROS_lib Convert_lib ${PROJECT_NAME}
)
target_link_libraries(mono
   ROS_lib Convert_lib ${PROJECT_NAME}
)
target_link_libraries(stereo_inertial
   ROS_lib Convert_lib ${PROJECT_NAME}
)
target_link_libraries(map_converter
   ROS_lib Convert_lib ${PROJECT_NAME}
)

# If RealSense SDK is found the library is added and its examples compiled
# if(realsense2_FOUND)
#    target_link_libraries(${PROJECT_NAME}_node
#     ${realsense2_LIBRARY}
#     )
# endif()
