#include <ros/ros.h>
#include <ros/package.h>
#include <moveit_msgs/PlanningScene.h> 
#include <geometry_msgs/Point.h>

#include <string>
#include <vector>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// moveit_msgs/CollisionObject Message:
// http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/CollisionObject.html

class Obstacle {
   public:
    std::string name_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;    //Linear velocity x, y, z 
    double rad_;
    Eigen::Vector3d center_; //For dynamic obstacles moving around a center with angular speed
    bool is_dynamic_;        //For dynamic obstacles moving around a center with angular speed
    double angular_speed_;   //For dynamic obstacles moving around a center with angular speed
    int direction_;          //For dynamic obstacles moving around a center with angular speed

   public:
    // Constructors
    Obstacle(const std::string& name, const Eigen::Vector3d& pos,
             const Eigen::Vector3d& vel, double rad, bool is_dynamic , // = false
             double angular_speed ) // = 0true
        : name_{name}, pos_{pos}, vel_{vel}, rad_{rad}, center_{pos},
          is_dynamic_{is_dynamic}, angular_speed_{angular_speed} {direction_  = (std::rand() % 2 == 0) ? -1 : 1;};

    Obstacle(const Eigen::Vector3d& pos, double rad)
        : pos_{pos}, rad_{rad}, vel_{0, 0, 0}, name_{""}, center_{pos},
          is_dynamic_{false}, angular_speed_{0.0} {};

    Obstacle(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
             double rad, bool is_dynamic = false, double angular_speed = 0.0)
        : pos_{pos}, rad_{rad}, vel_{vel}, name_{""}, center_{pos},
          is_dynamic_{is_dynamic}, angular_speed_{angular_speed} {};

    Obstacle() : pos_{0, 0, 0}, rad_{0}, vel_{0, 0, 0}, name_{""},
                 center_{0, 0, 0}, is_dynamic_{false}, angular_speed_{0.0} {};

    std::string getName() const { return name_; };
    Eigen::Vector3d getPosition() const { return pos_; };
    void setPosition(const Eigen::Vector3d& pos) { pos_ = pos; }
    void setVelocity(const Eigen::Vector3d& vel) { vel_ = vel; }
    Eigen::Vector3d getVelocity() const { return vel_; };
    double getRadius() const { return rad_; };
    void setDynamic(bool is_dynamic) { is_dynamic_ = is_dynamic; }
    bool isDynamic() const { return is_dynamic_; }
    void setCenter(const Eigen::Vector3d& center) { center_ = center; }
    Eigen::Vector3d getCenter() const { return center_; }
    void setAngularSpeed(double angular_speed) { angular_speed_ = angular_speed; }
    double getAngularSpeed() const { return angular_speed_; }

    // Update position for dynamic obstacles
void updatePosition(double delta_time)  //TODO Define more complex scenarios
{
    if (is_dynamic_)  // if is_dynamic, movement traj is a circle 
    {
        // set traj center as position 
        if (center_.isZero())  center_ = pos_; 
        double angle = direction_* angular_speed_ * delta_time;
        //double angle =  angular_speed_ * delta_time;
        Eigen::Vector3d offset = pos_ - center_;
        if (offset.norm() < 1e-6) 
        {
            offset = 2* Eigen::Vector3d(rad_, 0, 0); 
        }
        
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d rotated_offset = rotation * offset;
        pos_ = center_ + rotated_offset;
    } 
    else 
    {
        pos_ += vel_ * delta_time; // obstacle, linear movement 
        //pos_ = pos_; // static obstacle 
    }
}

};

Eigen::Vector3d readVector3d(const YAML::Node& node) 
{
    auto vec = node.as<std::vector<double>>();
    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

std::vector<Obstacle> readObstacles(const YAML::Node& node) 
{
    std::vector<Obstacle> obstacles;
    for (const auto& obstacle : node) {
        std::string name = obstacle["name"].as<std::string>();
        Eigen::Vector3d position = readVector3d(obstacle["position"]);
        Eigen::Vector3d velocity = readVector3d(obstacle["velocity"]);
        double radius = obstacle["radius"].as<double>();
        bool is_dynamic = obstacle["is_dynamic"].as<bool>();
        double angular_speed = obstacle["angular_speed"] ? obstacle["angular_speed"].as<double>() : 0.0;

        obstacles.emplace_back(name, position, velocity, radius, is_dynamic, angular_speed);
    }
    return obstacles;
}


void publishObstacles(ros::Publisher& pub, const std::vector<Obstacle>& obstacles) 
{
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true; 

    for (const auto& obstacle : obstacles) 
    {
        
        moveit_msgs::CollisionObject collision_obj;
        collision_obj.header.frame_id = "map";
        collision_obj.id = obstacle.name_;
        collision_obj.operation = moveit_msgs::CollisionObject::ADD;


        geometry_msgs::Pose pose;
        
        pose.position.x = obstacle.pos_.x();
        pose.position.y = obstacle.pos_.y();
        pose.position.z = obstacle.pos_.z();
        pose.orientation.w = 1.0; 
        collision_obj.primitive_poses.push_back(pose);

        
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::SPHERE;
        primitive.dimensions.push_back(obstacle.rad_);
        collision_obj.primitives.push_back(primitive);

        planning_scene_msg.world.collision_objects.push_back(collision_obj);
    }

    pub.publish(planning_scene_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_obstacle_publisher");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("robot_env_publisher");
    YAML::Node obstacles_yaml = YAML::LoadFile(package_path + "/config/obstacles_1.yaml");
    std::vector<Obstacle> obstacles = readObstacles(obstacles_yaml["obstacles"]);

    ros::Publisher obstacle_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Point>("goal_position", 10);

    //get goal position from yaml file
    std::string package_path_multi_agent = ros::package::getPath("multi_agent_vector_fields");
    YAML::Node start_goal = YAML::LoadFile(package_path_multi_agent + "/config/start_goal.yaml");
    Eigen::Vector3d goal_pos = readVector3d(start_goal["goal_pos"]);

    geometry_msgs::Point goal_msg;
    goal_msg.x = goal_pos.x();
    goal_msg.y = goal_pos.y();
    goal_msg.z = goal_pos.z();

    ros::Rate rate(10);
    while (ros::ok()) 
    {
        goal_pub.publish(goal_msg);
        
        for (auto& obstacle : obstacles) 
        {
            obstacle.updatePosition(0.1); 
        }
        publishObstacles(obstacle_pub, obstacles);
        //ROS_INFO("Publish obstacle planning_scene");

        rate.sleep();
    }

    return 0;
}
