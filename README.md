# Prerequis

## ROS

Installer ROS noetic (testé sur Ubuntu 20.04 et pop_os! 20.04):
http://wiki.ros.org/noetic/Installation/Ubuntu

### Catkin workspace

Créer un espace catkin pour l'installation de certains packages :
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Sourcer le workspace

```bat
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
ou directement modifier le fichier

```bat
cd 
nano .bashrc
```

Ajouter cette ligne
```bash

source ~/catkin_ws/devel/setup.bash
```

## RPLidar
Pour utiliser ce package avec le Lidar A2M8, installation du package ROS (suivre les isntructions https://github.com/robopeak/rplidar_ros)
```bat
cd ~/catkin_ws/src
git clone https://github.com/robopeak/rplidar_ros.git
cd ..
catkin_make
```

Pour tester que le Lidar fonctionne :
```bat
roslaunch rplidar_ros view_rplidar.launch 
```
## Navigation

```bat
sudo apt install ros-noetic-navigation
```

## Hector SLAM

```bat
sudo apt install ros-noetic-hector-slam
```

## Laser scan matcher

```bat
sudo apt install ros-noetic-laser-scan-matcher
```


## Teb local planner

```bat
sudo apt install ros-noetic-teb-local-planner
```

## Explore lite

```bat 
sudo apt install ros-noetic-explore-lite
```

## Rtabmap 

erreur

## Realsense D435
https://github.com/IntelRealSense/realsense-ros/blob/development/README.md#installation-instructions


```bat
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```
et l'installation du sdk
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

### Aruco OpenCV

```bat
pip install imutils
pip install pyrealsense2
```

Utliser un marker du dictionnaire 5x5 
```bat
python3 opencv_marker.py 
```



## Configuration reseau 

Pour configurer le réseau ROS comme dans le rapport, modification des fichiers .bashrc comme précédement :

Sur le PC Master :
```bash
export ROS_MASTER_URI=http://MASTER_IP:11311/
export ROS_IP=MASTER_IP
```

Sur la voiture :
```bash
export ROS_MASTER_URI=http://MASTER_IP:11311/
export ROS_IP=CAR_IP
```

Il faut aussi mofifier les IP dans les programmes Python
# ens_voiture_autonome

## Installation

Pour installer ce package, même procédure que pour le Lidar
```bat
cd ~/catkin_ws/src
git clone https://github.com/see-saw28/ens_voiture_autonome.git
cd ..
catkin_make
```

### Tester l'installation

Dans un premier terminal 
```bat
roslaunch ens_voiture_autonome localization.launch sim:=true
```
Dans un second terminal 
```bat
roscd ens_voiture_autonome/bag/
rosbag play 2022-03-25-16-21-15.bag --clock -l

```

## Car setup

```bat
roslaunch ens_voiture_autonome car_launch.launch ass:=true mapping:=false
```
## Param

sim : true si on utilise un bag, false pour lancer le lidar

map : nom de la carte utilisée pour la localisation (la carte doit se situer dans le dossier /map)

## Mapping

```bat
roslaunch ens_voiture_autonome mapping.launch method=hector sim:=false
```

### Hector SLAM
Cartographie avec hector et le Lidar :
```bat
roslaunch ens_voiture_autonome hector.launch sim:=false
```
### Rtab

#### With odometry (Realsense T265)
```bat
roslaunch ens_voiture_autonome rtab_odom.launch sim:=false
```

#### Without odometry
```bat
roslaunch ens_voiture_autonome rtab.launch sim:=false
```
#### Save the map

Pour sauvegarder la carte générée par Hector SLAM 
```bat
rosrun ros_server ros_saver -f nom_carte
```

#### Load a map

Pour sauvegarder la carte générée par Hector SLAM 
```bat
rosrun ros_server ros_server nom_carte.yaml
```

## Localization
```bat
roslaunch ens_voiture_autonome localization.launch method=t265 sim:=false
```
### With odometry (Realsense T265)
```bat
roslaunch ens_voiture_autonome localization_t265.launch sim:=false map:=nom_carte
```

### Without odometry
```bat
roslaunch ens_voiture_autonome localization_hector.launch sim:=false map:=nom_carte
```

## Navigation

```bat
roslaunch ens_voiture_autonome navigation.launch sim:=false map:=nom_carte
```

## Exploration

```bat
roslaunch ens_voiture_autonome exploration.launch sim:=false 
```
# Teleop 

## Keyboard teleop

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=keyboard_cmd _speed:=1.0 _turn:=0.3
```
### Published topic
* /keyboard_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

## DualShock 4

Read the controller input with Pygame and publish a custom message with the buttons and joystick state and rising edge of buttons.

```bash
rosrun ens_voiture_autonome DS4_ROS.py
```
### Published topic
* /DS4_input (ens_voiture_autonome/DS4)

## Central controller

```bash
rosrun ens_voiture_autonome joy_to_cmd_vel.py
```

* Calculate the speed command
* Check safety flag
* Control backward motion in stuck situation

### Subscribed topics
* /DS4_input (ens_voiture_autonome/DS4)
* /keyboard_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /pure_pursuit_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /stanley_control_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /local_steering_controller_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /dwa_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /move_base_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /follow_the_gap_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /AEB ([std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
* /collision ([std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
* /stuck ([std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

### Published topic
* /cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

# Reactive method

## Follow the gap

### Subscribed topic 

* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))

### Published topics

* /ftg_scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))
* /follow_the_gap_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /follow_the_gap_marker ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
* /follow_the_gap_obstacle_marker ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

### Parameters

* direct_steering : bool


#### Car parameters
* wheelbase_length : float : wheelbase size of the car in meter
* max_steering_angle : float : maximum steering angle in radian
* radius : float : width of the car in meter

#### Lidar processing parameters
* max_distance : float
* cut_angle : float
* conv_width : int

#### Speed control parameters
* max_velocity
* obstacle_velocity
* near_distance
* max_velocity_distance

# Path planning

## Minimum curvature path by TUMFTM

Integration of the MCP algorithm of TUMFTM for ROS

ROS map -> MCP Path -> ROS Path (via Path tools)

+ generate the centerline of a track

### Parameters  
* save : bool
* save_centerline : bool :
* flip : bool :change the direction of rotation of the path
* rolling_number : int : change the starting point on the path

# Path tracking

## Pure Pursuit

### Subscribed topics
* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
* /syscommand ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
* /mcp_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* /scan (Optional, [sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))

### Published topics
* /pure_pursuit_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /pure_pursuit_look_ahead ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
* /pure_pursuit_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
* /collision (Optional, [std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
* /collision_check (Optional, [visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

### Parameters

#### Car parameters
* wheelbase_length : float : wheelbase size of the car in meter
* max_steering_angle : float : maximum steering angle in radian
* radius : float : width of the car in meter

#### Look ahead parameters
* look_ahead_dist : float : static look ahead distance 
* k : float : look foward term

`dyn_look_ahead_dist = look_ahead_dist + k*v`

#### Obstacle detection parameters
* look_ahead_dist_obstacle : float : static look ahead distance 
* k_obstacle : float : look foward term
`dyn_look_ahead_dist = look_ahead_dist + k*v`
* detect_collision : bool : when set to true, if an obstacle is detected then use the `local_steering_controller_cmd`

## Stanley control

### Subscribed topics
* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
* /syscommand ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
* /mcp_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

### Published topics
* /stanley_control_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /stanley_controller_look_ahead ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

### Parameters

#### Car parameters
* wheelbase_length : float : wheelbase size of the car in meter
* max_steering_angle : float : maximum steering angle in radian
* radius : float : width of the car in meter

#### Stanley parameters
* k_sc : float :
* k_soft : float :
* k_steer : float :

## Dynamic window approach (adapted for car like robot)

### Subscribed topics
* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
* /syscommand ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
* /mcp_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))

### Published topics
* /dwa_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /daw_look_ahead ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
* /dwa_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))

### Parameters

#### Car parameters
* wheelbase_length : float : wheelbase size of the car in meter
* robot_width : float : width of the car in meter
* robot_length : float : length of the car in meter
* max_steering_angle : float : maximum steering angle in radian
* steering_speed
* max_yaw_rate
* max_speed
* min_speed
* max_accel


#### DWA parameters
* steering_resolution : float : steering resolution for the dynamic window
* v_resolution : float : speed resolution for the dynamic window
* predict_time : float : predict time for trajectory prediction
* dt : float : time step for trajectory prediction
* to_goal_cost_gain : float : gain for the to goal cost function
* speed_cost_gain : float : gain for the speed cost function
* obstacle_cost_gain : float : gain for the obstacle cost function

#### Look ahead parameters
* look_ahead_dist : float : static look ahead distance 
* k : float : look foward term

## Local steering controller (derivated from DWA only for steering)

### Subscribed topics
* /amcl_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
* /syscommand ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))
* /mcp_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))
* /collision (optional, [std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html)

### Published topics
* /local_steering_controller_cmd ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /local_steering_controller_look_ahead ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))
* /local_steering_controller_path ([nav_msgs/Path](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/Path.html))

### Parameters

#### Car parameters
* wheelbase_length : float : wheelbase size of the car in meter
* robot_width : float : width of the car in meter
* robot_length : float : length of the car in meter
* max_steering_angle : float : maximum steering angle in radian
* steering_speed 
* max_yaw_rate

#### DWA parameters
* steering_resolution : float : steering resolution for the dynamic window
* predict_time : float : predict time for trajectory prediction
* dt : float : time step for trajectory prediction
* to_goal_cost_gain : float : gain for the to goal cost function
* obstacle_cost_gain : float : gain for the obstacle cost function

#### Look ahead parameters
* look_ahead_dist : float : static look ahead distance 
* k : float : look foward term

# Safety

## AEB (automatic emergency breaking)

### Subscribed topics
* /cmd_vel or /vel ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))


### Published topics
* /AEB ([std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
* /aeb_marker ([visualization_msgs/Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html))

## Stuck detector

### Subscribed topics
* /camera/odom/sample ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
* /scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/LaserScan.html))


### Published topic
* /stuck ([std_msgs/Bool](http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Bool.html))
