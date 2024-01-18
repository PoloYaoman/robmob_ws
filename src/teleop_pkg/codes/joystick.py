#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
rapport = 0.2

def joystick_callback(data):
    # Créer un message Twist à partir des axes du joystick
    twist = Twist()
    twist.linear.x = data.axes[1]*rapport # Utiliser l'axe vertical pour la vélocité linéaire
    twist.angular.z = data.axes[0]*rapport  # Utiliser l'axe horizontal pour la vélocité angulaire

    # Publier le message Twist sur le topic /cmd_vel
    pub.publish(twist)

def joystick_listener():
    global pub
    rospy.init_node('joystick_controller', anonymous=True)

    # Créer un éditeur pour publier sur /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Joystick controller node is ready.")

    # S'abonner au topic joy
    rospy.Subscriber("joy", Joy, joystick_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_listener()
    except rospy.ROSInterruptException:
        pass
