#ifndef ROBOT_ENV_PUBLISHER_PARSE_HELPER_HPP
#define ROBOT_ENV_PUBLISHER_PARSE_HELPER_HPP

#include <type_traits>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Helper to parse ROS parameters from parameter server
#define READ_PARAM(node_handle, node_name, param_name, param_variable) \
  if (!node_handle.getParam(param_name, param_variable)) { \
    ROS_ERROR_STREAM(node_name << ": Could not get parameter " << param_name); \
    return false; \
  } \
  else{ \
    ROS_INFO_STREAM(node_name << ": Getting parameter " << param_name << ": " << param_variable); \
  }

// Helper to parse YAML nodes

// Helper to detect Eigen vector types
template <typename T>
struct is_eigen_vector_type : std::false_type {};

// Specialize for Eigen vector types (e.g., Eigen::VectorXd, Eigen::Vector3d, etc.)
template <typename T, int Rows, int Options, int MaxRows>
struct is_eigen_vector_type<Eigen::Matrix<T, Rows, 1, Options, MaxRows, 1>> : std::true_type {};

// General template for non-Eigen types
template <typename T>
typename std::enable_if<!is_eigen_vector_type<T>::value, void>::type
YAMLPhraseWithAssert(const YAML::Node& node, const std::string& key, T& variable) {
    if (node[key]) {
        variable = node[key].as<T>();
    } else {
        throw std::runtime_error("YAML structure error: Missing key '" + key + "' in the YAML node: " + node.as<std::string>());
    }
}

// Specialization for Eigen vector types
template <typename T>
typename std::enable_if<is_eigen_vector_type<T>::value, void>::type
YAMLPhraseWithAssert(const YAML::Node& node, const std::string& key, T& variable, typename T::Index expected_size) {
    if (node[key]) {
        auto temp = node[key].as<std::vector<double>>();
        if (temp.size() != expected_size) {
            throw std::runtime_error("YAML structure error: Key '" + key + "' does not have the expected size of " +
                                     std::to_string(expected_size) + " in the YAML node: " + node.as<std::string>());
        }
        for (typename T::Index i = 0; i < expected_size; ++i) {
            variable[i] = temp[i];
        }
    } else {
        throw std::runtime_error("YAML structure error: Missing key '" + key + "'" +
                                 " in the YAML node: " + node.as<std::string>());
    }
}

// General template for non-Eigen types
template <typename T>
typename std::enable_if<!is_eigen_vector_type<T>::value, void>::type
YAMLPhraseOptional(const YAML::Node& node, const std::string& key, T& variable, const T& default_assignment) {
    if (node[key]) {
        variable = node[key].as<T>();
    } else {
        std::cout << "YAML structure warning: Not found key '" << key << "' in the YAML node: "
                  << node.as<std::string>() << ". Using default value: " << default_assignment << std::endl;
        variable = default_assignment;
    }
}

// Specialization for Eigen vector types
template <typename T>
typename std::enable_if<is_eigen_vector_type<T>::value, void>::type
YAMLPhraseOptional(const YAML::Node& node, const std::string& key, T& variable, const T& default_assignment, typename T::Index expected_size) {
    if (node[key]) {
        auto temp = node[key].as<std::vector<double>>();
        if (temp.size() != expected_size) {
            throw std::runtime_error("YAML structure error: Key '" + key + "' does not have the expected size of " +
                                     std::to_string(expected_size) + " in the YAML node: " + node.as<std::string>());
        }
        for (typename T::Index i = 0; i < expected_size; ++i) {
            variable[i] = temp[i];
        }
    } else {
        std::cout << "YAML structure warning: Not found key '" << key << "' in the YAML node: "
                  << node.as<std::string>() << ". Using default value." << std::endl;
        variable = default_assignment;
    }
}

#endif // ROBOT_ENV_PUBLISHER_PARSE_HELPER_HPP