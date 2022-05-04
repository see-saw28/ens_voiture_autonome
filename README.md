# ens_voiture_autonome

## Installation

cloner ce git dans le dossier src du catkin workspace
puis catkin_make
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

```bat
roslaunch ens_voiture_autonome mapping.launch sim:=false
```

#### Sauvegarde de la carte

```bat
rosrun ros_server ros_saver -f nom_carte
```

## Localisation

```bat
roslaunch ens_voiture_autonome localization.launch sim:=false map:=map1
```

## Navigation

```bat
roslaunch ens_voiture_autonome navigation.launch sim:=false map:=map1
```

## Exploration

```bat
roslaunch ens_voiture_autonome exploration.launch sim:=false 
```
