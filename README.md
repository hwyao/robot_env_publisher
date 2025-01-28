# robot_env_publisher

using moveit_msgs [`moveit_msgs/PlanningScene`] [`PlanningSceneWorld`] [`CollisionObject`]

http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/PlanningScene.html

![image](https://github.com/user-attachments/assets/8214f50c-4957-4bc5-a516-14657fd0f220)

### In order to run it:

1.  install ROS moveit package: `sudo apt install ros-noetic-moveit`

2.  `roslaunch robot_env_publisher obstacles_publisher.launch`

### In order to echo topic:

1.  `rostopic echo /planning_scene`

### In order to visualise:

1.  make sure that you have already install [moveit](https://moveit.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin) and download the example code

![image](https://github.com/user-attachments/assets/48e465de-1366-49bb-844d-af03c3ecb23f)

4.  `roslaunch panda_moveit_config demo.launch `
5.  
![image](https://github.com/user-attachments/assets/cd3b5ac2-b2ca-481a-9b1a-1362866dbd90)


