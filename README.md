# ens_voiture_autonome

## Installation


## Mapping

roslaunch ens_voiture_autonome mapping.launch sim:=false

### Sauvegarde de la carte

rosrun ros_server ros_saver -f nom_carte

## Localisation

roslaunch ens_voiture_autonome localization.launch sim:=false map:=map1

## Navigation

roslaunch ens_voiture_autonome navigation.launch sim:=false map:=map1

## Exploration

roslaunch ens_voiture_autonome exploration.launch sim:=false 
