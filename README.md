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