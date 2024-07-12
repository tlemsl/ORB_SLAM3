#include <MapConverter.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

void savePlaneData(const Eigen::Vector3f& normal, const Eigen::Vector3f& point, const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "plane_normal" << YAML::Value << YAML::Flow << YAML::BeginSeq << normal.x() << normal.y() << normal.z() << YAML::EndSeq;
    out << YAML::Key << "plane_point" << YAML::Value << YAML::Flow << YAML::BeginSeq << point.x() << point.y() << point.z() << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(filename);
    if (fout.is_open()) {
        fout << out.c_str();
        fout.close();
        std::cout << "Plane data saved to " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
}

// Function to load plane data from a YAML file
bool loadPlaneData(const std::string& filename, Eigen::Vector3f& normal, Eigen::Vector3f& point) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["plane_normal"] && config["plane_point"]) {
            auto normal_node = config["plane_normal"];
            auto point_node = config["plane_point"];
            normal = Eigen::Vector3f(normal_node[0].as<float>(), normal_node[1].as<float>(), normal_node[2].as<float>());
            point = Eigen::Vector3f(point_node[0].as<float>(), point_node[1].as<float>(), point_node[2].as<float>());
            return true;
        } else {
            std::cerr << "Error: Missing keys in YAML file." << std::endl;
            return false;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error: Failed to load YAML file: " << e.what() << std::endl;
        return false;
    }
}

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