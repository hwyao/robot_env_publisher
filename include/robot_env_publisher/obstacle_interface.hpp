#ifndef ROBOT_ENV_PUBLISHER_OBSTACLE_INTERFACE_HPP
#define ROBOT_ENV_PUBLISHER_OBSTACLE_INTERFACE_HPP

#include "robot_env_publisher/parse_helper.hpp"

#include <moveit_msgs/CollisionObject.h>

namespace robot_env_publisher {

/**
 * @brief The ObstacleInterface class
 * 
 * This interface is used to get the obstacle state callback. The data source that implements this interface
 * should fill in the CollisionObject message with the obstacle information.
 */
class ObstacleInterface {
public:
    virtual ~ObstacleInterface() = default;
    
    /**
     * @brief Get the Obstacle State Callback object
     * 
     * @param collision_object The reference to the CollisionObject message. For each of the different
     * data source that implement this interface, the data source should fill in this data (to be injected into the
     * planning scene).
     * @param clock The current time used for updating the obstacle state. Defaults to 0.
     */
    virtual void getObstacleStateCallback(moveit_msgs::CollisionObject& collision_object, double clock = 0) = 0;

    /**
     * @brief Generate a report of the obstacle information.
     * 
     * @param report A reference to a string where the obstacle information will be stored.
     */
    virtual void obstacleInfoReport(std::string& report) const = 0;
};

class ObstaclePhraseFromYamlNode : public ObstacleInterface {
public:
    /**
     * @brief Construct a new Obstacle Phrase From Yaml Node object
     * 
     * @param yaml_node The YAML node that contains a phrase of one single YAML structure that represents the
     * obstacle.
     * @param frequency The frequency of the obstacle. This is used to drive the obstacle move continuously when
     * calling the getObstacleStateCallback function.
     */
    ObstaclePhraseFromYamlNode(const YAML::Node& yaml_node);

    /**
     * @brief Destroy the Obstacle Phrase From Yaml Node object
     * 
     */
    ~ObstaclePhraseFromYamlNode() override = default;

    /**
     * @brief Get the Obstacle State Callback object
     * 
     * @param collision_object The reference to the CollisionObject message. For each of the different
     * data source that implement this interface, the data source should fill in this data (to be injected into the
     * planning scene).
     * @param clock The current time used for updating the obstacle state. Defaults to 0.
     */
    void getObstacleStateCallback(moveit_msgs::CollisionObject& collision_object, double clock = 0) override;

    /**
     * @brief Generate a report of the obstacle information.
     * 
     * @param report A reference to a string where the obstacle information will be stored.
     */
    void obstacleInfoReport(std::string& report) const override;

protected:
    /**
     * @brief Update the position of the obstacle
     * 
     * Update the position state of the obstacle. 
     */
    void updatePosition();

private:
    // data phrased from YAML node
    std::string name_;
    Eigen::Matrix4d transform_initial_;
    Eigen::Vector3d velocity_w_;
    Eigen::Vector3d velocity_v_;
    bool use_velocity_as_twist_;
    std::string primitiveType; 
    std::vector<double> primitiveDimension; 

    // state data
    Eigen::Matrix4d transform_now_;
    double time_;     // The time of the obstacle update
};

} // namespace robot_env_publisher

#endif // ROBOT_ENV_PUBLISHER_OBSTACLE_INTERFACE_HPP