#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Fréquence de mise à jour (10 Hz)
        self.forward_speed = 0.2
        self.min_dist_to_obstacle = 0.17 # Seuil de distance pour la détection des obstacles
        self.obstacle_detected = False
        self.obstacle_detected_left = False
        self.obstacle_detected_ahead = False
        # Initialisation des commandes de mouvement du robot
        self.move_cmd = Twist()
        self.angle_to_obstacle = 0.0
        
        self.espace_gauche = False

    def lidar_callback(self, data):
        # Obtenir les données du Lidar
        scan = data.ranges

        # Vérifier s'il y a des obstacles
        obstacle_detected = False
        min_distance = min(x for x in scan if x > 0)
        if min_distance < self.min_dist_to_obstacle:
            obstacle_detected = True

        # Faire quelque chose en fonction de la détection des obstacles
        if obstacle_detected:
            # Calculer l'angle vers l'obstacle le plus proche
            angle_min = data.angle_min
            angle_increment = data.angle_increment
            index_min = scan.index(min_distance)
            self.angle_to_obstacle = angle_min + index_min * angle_increment
    
            rospy.loginfo("obstacle détécté !")  
            rospy.loginfo(self.angle_to_obstacle)  
            if (self.angle_to_obstacle >= 3.85) & (self.angle_to_obstacle <= 5): # Obstacle à gauche
                self.obstacle_detected_left = True 
            else:
                self.obstacle_detected_left = False
            if (self.angle_to_obstacle > 2.12) & (self.angle_to_obstacle <= 3.84):
                self.obstacle_detected_ahead = True 
            else:
                self.obstacle_detected_ahead = False

        else:
            self.obstacle_detected_left = False
            self.obstacle_detected_ahead = False
        
        rospy.loginfo("Gauche "+ str(self.obstacle_detected_left))
        rospy.loginfo("Devant "+ str(self.obstacle_detected_ahead))
        #Déplacements
        if (not self.obstacle_detected_left) & (not self.obstacle_detected_ahead) :
            #tourne à gauche
            rospy.loginfo("tourne à gauche !") 
            self.move_cmd.linear.x = 0.01
            self.move_cmd.angular.z = 0.3
            self.cmd_vel_pub.publish(self.move_cmd)
        else:
            if (min_distance >= 0.13):
                if (self.obstacle_detected_left) & (not self.obstacle_detected_ahead) : 
                    #va tout droit
                    rospy.loginfo("Tout droit !") 
                    self.move_cmd.linear.x = 0.1
                    self.move_cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.move_cmd)
                elif (not self.obstacle_detected_left) & (self.obstacle_detected_ahead) :
                    #tourne à gauche
                    rospy.loginfo("tourne à gauche !") 
                    self.move_cmd.linear.x = 0.01
                    self.move_cmd.angular.z = 0.3
                    self.cmd_vel_pub.publish(self.move_cmd)
                else:
                    #tourne à droite
                    rospy.loginfo("tourne à droite !") 
                    self.move_cmd.linear.x = 0.01
                    self.move_cmd.angular.z = -0.9
                    self.cmd_vel_pub.publish(self.move_cmd)
            else:
                rospy.loginfo("tourne à droite !") 
                self.move_cmd.linear.x = 0.01
                self.move_cmd.angular.z = -0.9
                self.cmd_vel_pub.publish(self.move_cmd)
                
   
    def run(self):
        # Boucle infinie jusqu'à ce que le noeud soit arrêté
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass