#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap



def load_map():
    rospy.init_node('load_map_node', anonymous=True)

    rospy.wait_for_service('/static_map')  # Assurez-vous que le service /static_map est disponible
    rospy.loginfo("static map reached")

    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        response = get_map()
        map_data = response.map

        # Vous pouvez maintenant utiliser map_data comme bon vous semble
        # Par exemple, imprimer la largeur et la hauteur de la carte
        print("Largeur de la carte :", map_data.info.width)
        print("Hauteur de la carte :", map_data.info.height)

    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    load_map()
