#include "robot_env_publisher/obstacle_interface.hpp"

#include <vector>
#include <memory>
#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv) {
    std::string node_name = "dynamic_obstacle_publisher";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    
    // Load the ROS parameter
    std::string yaml_file_name;
    int frequency;
    bool enable_marker_visualization;
    READ_PARAM(nh, node_name, node_name + "/env_scene", yaml_file_name);
    READ_PARAM(nh, node_name, node_name + "/frequency", frequency);
    READ_PARAM(nh, node_name, node_name + "/enable_marker_visualization", enable_marker_visualization);

    // Load the YAML file and parse the obstacles
    Eigen::Vector3d goal_position;
    Eigen::Vector4d goal_rotation;
    std::vector<std::unique_ptr<robot_env_publisher::ObstaclePhraseFromYamlNode>> obstacles;
    
    // Check if the YAML file exists
    std::string package_path = ros::package::getPath("robot_env_publisher");
    std::string yaml_file_path = package_path + "/scene/" + yaml_file_name;
    if (!std::filesystem::exists(yaml_file_path)) {
        ROS_ERROR_STREAM("YAML file not found: " << yaml_file_path);
        return -1;
    }

    // Load the YAML file
    YAML::Node env_yaml = YAML::LoadFile(yaml_file_path);
    
    // Load the initial configuration as ROS parameter
    std::vector<double> initial_config;
    YAMLPhraseWithAssert(env_yaml, "initial_configuration", initial_config);
    nh.setParam("/initial_configuration", initial_config);

    // Load the goal position and rotation
    YAMLPhraseWithAssert(env_yaml["goal"], "position", goal_position, 3);
    YAMLPhraseWithAssert(env_yaml["goal"], "orientation", goal_rotation, 4);
    if (std::abs(goal_rotation.norm() - 1.0) > 1e-6) {
        throw std::runtime_error("YAML structure error: Key 'goal.orientation' does not represent a normalized quaternion. Norm: " + std::to_string(goal_rotation.norm()));
    }

    // Load the obstacles
    YAML::Node obstacles_yaml;
    YAMLPhraseWithAssert(env_yaml, "obstacles", obstacles_yaml);
    if (obstacles_yaml.IsSequence()) {
        for (const auto& obstacle_yaml : obstacles_yaml) {
            robot_env_publisher::ObstaclePhraseFromYamlNode obstacle(obstacle_yaml, 10);
            obstacles.push_back(std::make_unique<robot_env_publisher::ObstaclePhraseFromYamlNode>(obstacle));
        }
    }

    // Display the information of the loaded information
    ROS_INFO_STREAM("dynamic_obstacle_publisher: Loaded goal position: [" 
                    << goal_position[0] << ", " 
                    << goal_position[1] << ", " 
                    << goal_position[2] << "]");
    ROS_INFO_STREAM("dynamic_obstacle_publisher: Loaded goal orientation (quaternion): [" 
                    << goal_rotation[0] << ", " 
                    << goal_rotation[1] << ", " 
                    << goal_rotation[2] << ", " 
                    << goal_rotation[3] << "]");

    // Display the information of the loaded obstacles
    ROS_INFO_STREAM("dynamic_obstacle_publisher: Number of obstacles loaded: " << obstacles.size());
    for (size_t i = 0; i < obstacles.size(); ++i) {
        ROS_INFO_STREAM("Obstacle " << i + 1 << ":");
        ROS_INFO_STREAM("  Name: " << obstacles[i]->getName());
        ROS_INFO_STREAM("  Primitive Type: " << obstacles[i]->getPrimitiveType());

        const auto& dimensions = obstacles[i]->getPrimitiveDimension();
        if (dimensions.size() > 3) {
            ROS_WARN_STREAM("Obstacle " << i + 1 << " has more than 3 dimensions. Only the first 3 will be displayed.");
        }

        ROS_INFO_STREAM("  Primitive Dimensions: ["
                        << (dimensions.size() > 0 ? std::to_string(dimensions[0]) : "N/A") << ", "
                        << (dimensions.size() > 1 ? std::to_string(dimensions[1]) : "N/A") << ", "
                        << (dimensions.size() > 2 ? std::to_string(dimensions[2]) : "N/A") << "]");
    }

    // set up the publishers
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
    ros::Publisher obstacle_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Publisher obstacle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10);
    
    // pre allocate the message to be published
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "world";
    goal_msg.pose.position.x = goal_position[0];
    goal_msg.pose.position.y = goal_position[1];
    goal_msg.pose.position.z = goal_position[2];
    goal_msg.pose.orientation.x = goal_rotation[0];
    goal_msg.pose.orientation.y = goal_rotation[1];
    goal_msg.pose.orientation.z = goal_rotation[2];
    goal_msg.pose.orientation.w = goal_rotation[3];

    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = false;

    // start the main node
    ros::Rate rate(frequency);
    while (ros::ok()) 
    {
        // update the goal
        goal_pub.publish(goal_msg);

        // update the obstacles as scene
        planning_scene_msg.world.collision_objects.clear();
        for (size_t i = 0; i < obstacles.size(); ++i) 
        {
            // prepare the planning scene message
            moveit_msgs::CollisionObject collision_obj;
            obstacles[i]->getObstacleStateCallback(collision_obj);
            planning_scene_msg.world.collision_objects.push_back(collision_obj);
        }

        obstacle_pub.publish(planning_scene_msg);

        if (enable_marker_visualization) {
            visualization_msgs::MarkerArray obstacle_vis_markers;
            for (size_t i = 0; i < obstacles.size(); ++i) {
                visualization_msgs::Marker obstacle_vis_marker;
                obstacle_vis_marker.header.frame_id = planning_scene_msg.world.collision_objects[i].header.frame_id;
                obstacle_vis_marker.header.stamp = ros::Time::now();
                obstacle_vis_marker.ns = planning_scene_msg.world.collision_objects[i].id;
                obstacle_vis_marker.id = static_cast<int>(i);

                // Check if there is exactly one primitive
                if (planning_scene_msg.world.collision_objects[i].primitives.size() == 1) {
                    const auto& primitive = planning_scene_msg.world.collision_objects[i].primitives[0];
                    switch (primitive.type) {
                        case shape_msgs::SolidPrimitive::BOX:
                            obstacle_vis_marker.type = visualization_msgs::Marker::CUBE;
                            obstacle_vis_marker.scale.x = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
                            obstacle_vis_marker.scale.y = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                            obstacle_vis_marker.scale.z = primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
                            break;
                        case shape_msgs::SolidPrimitive::SPHERE:
                            obstacle_vis_marker.type = visualization_msgs::Marker::SPHERE;
                            obstacle_vis_marker.scale.x = primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] * 2;
                            obstacle_vis_marker.scale.y = primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] * 2;
                            obstacle_vis_marker.scale.z = primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] * 2;
                            break;
                        case shape_msgs::SolidPrimitive::CYLINDER:
                            obstacle_vis_marker.type = visualization_msgs::Marker::CYLINDER;
                            obstacle_vis_marker.scale.x = primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] * 2;
                            obstacle_vis_marker.scale.y = primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] * 2;
                            obstacle_vis_marker.scale.z = primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
                            break;
                        default:
                            ROS_WARN_THROTTLE(1.0, "Unknown primitive type in collision object.");
                            obstacle_vis_marker.type = visualization_msgs::Marker::CUBE; // Default to CUBE
                            obstacle_vis_marker.scale.x = 1.0;
                            obstacle_vis_marker.scale.y = 1.0;
                            obstacle_vis_marker.scale.z = 1.0;
                            break;
                    }

                    // Set position and orientation
                    obstacle_vis_marker.pose = planning_scene_msg.world.collision_objects[i].primitive_poses[0];

                    // Set color
                    obstacle_vis_marker.color.r = 0.0;
                    obstacle_vis_marker.color.g = 0.0;
                    obstacle_vis_marker.color.b = 1.0;
                    obstacle_vis_marker.color.a = 0.6; 

                    obstacle_vis_markers.markers.push_back(obstacle_vis_marker);
                } else {
                    ROS_WARN_THROTTLE(1.0, "Collision object must have exactly one primitive to be visualized.");
                }
            }

            obstacle_vis_pub.publish(obstacle_vis_markers);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}