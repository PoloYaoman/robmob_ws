penser à toujours avoir la manette branchée avant de lancer le docker
pour donner les droits au docker de lire la manette ajouter la ligne:
--device=/dev/input/js0 \
dans le strart_docker.bash

lancer avec el launch du teleop
afficher la map avec rviz

sauvegarder la map à la fin avec:
# Sauvegarder la carte au format PGM
rosrun map_server map_saver -f nom_de_carte

# Ou sauvegarder la carte au format YAML
rosrun map_server map_saver -f nom_de_carte.yaml

pour publier une map:
rosrun map_server map_server nom_map.yaml

voir les topic et &: rqt_graph

map_metadata: recuperer l'origine de la carte dans le tableau (a voir dans la doc car non sûr)
# IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
APT UPDATE
APT UPGRADE -Y
APT INSTALL PYTHON3-TK -Y
# IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT

ROBOT IRL:
Se connecter au wifi du robot
connexion en ssh au robot: user@192.168.0.200
mot de passe : #user

nano ~/.bashrc
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=192.168.0.100

lancer dans le robot: roslaunch minilab_launch minilab_driver_hokuyo.launch
lancer dans le docker: roslaunch teleop_pkg robot.launch
have fun
