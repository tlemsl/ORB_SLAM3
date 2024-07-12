#ifndef MAPCONVERTER_H
#define MAPCONVERTER_H
#include <Eigen/Dense>

Eigen::Matrix4f getPlaneTransformation(const Eigen::Vector3f& normal, const Eigen::Vector3f& center);
void savePlaneData(const Eigen::Vector3f& normal, const Eigen::Vector3f& point, const std::string& filename);
bool loadPlaneData(const std::string& filename, Eigen::Vector3f& normal, Eigen::Vector3f& point);

#endif