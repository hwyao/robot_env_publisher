#include "robot_env_publisher/obstacle_interface.hpp"

#include <pinocchio/spatial/explog.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>

namespace robot_env_publisher {

ObstaclePhraseFromYamlNode::ObstaclePhraseFromYamlNode(const YAML::Node& yaml_node, int frequency)
    : frequency_(frequency), time_(0.0), transform_initial_(Eigen::Matrix4d::Identity()), transform_now_(Eigen::Matrix4d::Identity()) {
    
    // Check if the YAML node is a map
    if (!yaml_node.IsMap()) {
        throw std::runtime_error("YAML node is not a map: " + yaml_node.as<std::string>());
    }

    // Parse the YAML node
    YAMLPhraseWithAssert(yaml_node, "name", name_);

    Eigen::Vector3d position;
    YAMLPhraseWithAssert(yaml_node, "position", position, 3);
    transform_initial_.block<3, 1>(0, 3) = position;

    Eigen::Vector4d orientation;
    YAMLPhraseWithAssert(yaml_node, "orientation", orientation, 4);
    if (std::abs(orientation.norm() - 1.0) > 1e-6) {
        throw std::runtime_error("YAML structure error: Key 'obstacle[].orientation' does not represent a normalized quaternion. Norm: " + std::to_string(orientation.norm()));
    }
    transform_initial_.block<3, 3>(0, 0) = Eigen::Quaterniond(orientation[3], orientation[0], orientation[1], orientation[2]).toRotationMatrix();
    transform_now_ = transform_initial_;

    // Correct usage of YAMLPhraseOptional for Eigen vectors
    YAMLPhraseOptional(yaml_node, "velocity_w", velocity_w_, Eigen::Vector3d::Zero().eval(), 3);
    YAMLPhraseOptional(yaml_node, "velocity_v", velocity_v_, Eigen::Vector3d::Zero().eval(), 3);
    YAMLPhraseOptional(yaml_node, "use_velocity_as_twist", use_velocity_as_twist_, true);

    // Parse primitive type and dimensions
    YAMLPhraseWithAssert(yaml_node["description"], "type", primitiveType);
    YAMLPhraseWithAssert(yaml_node["description"], "dimensions", primitiveDimension);

    // Validate dimensions based on type
    if (primitiveType == "BOX" && primitiveDimension.size() != 3) {
        throw std::runtime_error("Invalid dimensions for BOX. Expected 3 dimensions (x, y, z). Provided: " + std::to_string(primitiveDimension.size()));
    } else if (primitiveType == "SPHERE" && primitiveDimension.size() != 1) {
        throw std::runtime_error("Invalid dimensions for SPHERE. Expected 1 dimension (radius). Provided: " + std::to_string(primitiveDimension.size()));
    } else if (primitiveType == "CYLINDER" && primitiveDimension.size() != 2) {
        throw std::runtime_error("Invalid dimensions for CYLINDER. Expected 2 dimensions (height, radius). Provided: " + std::to_string(primitiveDimension.size()));
    } else if (primitiveType == "CONE" && primitiveDimension.size() != 2) {
        throw std::runtime_error("Invalid dimensions for CONE. Expected 2 dimensions (height, radius). Provided: " + std::to_string(primitiveDimension.size()));
    }
}

void ObstaclePhraseFromYamlNode::getObstacleStateCallback(moveit_msgs::CollisionObject& collision_object) {
    collision_object.header.frame_id = "world";
    collision_object.id = name_;
    collision_object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::Pose pose;
    pose.position.x = transform_now_(0, 3);
    pose.position.y = transform_now_(1, 3);
    pose.position.z = transform_now_(2, 3);

    Eigen::Quaterniond quaternion(transform_now_.block<3, 3>(0, 0));
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    collision_object.primitive_poses.push_back(pose);

    shape_msgs::SolidPrimitive primitive;

    // Set primitive type based on parsed data
    if (primitiveType == "BOX") {
        primitive.type = shape_msgs::SolidPrimitive::BOX;
    } else if (primitiveType == "SPHERE") {
        primitive.type = shape_msgs::SolidPrimitive::SPHERE;
    } else if (primitiveType == "CYLINDER") {
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
    } else if (primitiveType == "CONE") {
        primitive.type = shape_msgs::SolidPrimitive::CONE;
    } else {
        throw std::runtime_error("Unsupported primitive type: " + primitiveType);
    }

    // Set primitive dimensions
    primitive.dimensions = primitiveDimension;

    collision_object.primitives.push_back(primitive);

    updatePosition();
}

void ObstaclePhraseFromYamlNode::updatePosition() {
    pinocchio::SE3 transform_initial(transform_initial_);
    pinocchio::SE3 transform_now(transform_now_);

    if (use_velocity_as_twist_) {
        pinocchio::Motion twist(velocity_v_, velocity_w_);
        transform_now = transform_initial * pinocchio::exp6(twist * time_);
    } else {
        // Apply rotation and position separately
        Eigen::Vector3d rotation_vector = velocity_w_ * time_;
        Eigen::Matrix3d rotation_matrix = pinocchio::exp3(rotation_vector);

        Eigen::Vector3d translation = velocity_v_ * time_;

        transform_now.rotation() = transform_initial.rotation() * rotation_matrix;
        transform_now.translation() = transform_initial.translation() + translation;
    }

    transform_now_ = transform_now.toHomogeneousMatrix();
    time_ += 1.0 / static_cast<double>(frequency_);
}

} // namespace robot_env_publisher