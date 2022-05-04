# Prerequis

## ROS

Installer ROS noetic (testé sur Ubuntu 20.04):
http://wiki.ros.org/noetic/Installation/Ubuntu

### Catkin workspace

Créer un espace catkin pour l'installation de certains packages :
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Sourcer le workspace et ROS
```bat
cd 
nano .bashrc
```

Ajouter ces lignes
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## RPLidar
Pour utiliser ce package avec le Lidar A2M8, installation du package ROS (suivre les isntructions https://github.com/robopeak/rplidar_ros)
```bat
cd catkin_ws/src
git clone https://github.com/robopeak/rplidar_ros.git
cd ..
catkin_make
```

Pour tester que le Lidar fonctionne :
```bat
roslaunch rplidar_ros view_rplidar.launch 
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
cd catkin_ws/src
git clone https://github.com/see-saw28/ens_voiture_autonome.git
cd ..
catkin_make
```

## Param

sim : true si on utilise un bag, false pour lancer le lidar

map : nom de la carte utilisée pour la localisation (la carte doit se situer dans le dossier /map)

## Mapping

Cartographie avec hector et le Lidar :
```bat
roslaunch ens_voiture_autonome mapping.launch sim:=false
```

#### Sauvegarde de la carte

Pour sauvegarder la carte générée par Hector SLAM 
```bat
rosrun ros_server ros_saver -f nom_carte
```

## Localisation

```bat
roslaunch ens_voiture_autonome localization.launch sim:=false map:=nom_carte
```

## Navigation

```bat
roslaunch ens_voiture_autonome navigation.launch sim:=false map:=nom_carte
```

## Exploration

```bat
roslaunch ens_voiture_autonome exploration.launch sim:=false 
```
