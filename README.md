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

-->
on peut recuperer la transformée de l'origine de l'occupancygrid à la map à l'aide de la map_metadata
on à la translation et la rotation
tf_listener pour recuperer la transformée entre map et base_link(robot)
ensuite on utilise le passage de map à la grille d'occupation
on peut faire les déplacements comme ça