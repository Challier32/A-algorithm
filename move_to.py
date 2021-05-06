from rostron_ia_ms.utils.world import World
import math
from math import sin, cos, pi
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rostron_interfaces.msg import Robot, Robots
from rostron_interfaces.msg import Order, Hardware

class MoveTo(Node):
    poseRobots= Robots()
    def __init__(self):
        super().__init__("move_to")
        self.subscription = self.create_subscription(
            Robots,
            '/yellow/allies',
            self.listener_robots,
            1
        )
        self.subscription
        
        self.publisher = self.create_publisher(Order,'/yellow/order', 1)

    def listener_robots(self, robots):
        global poseRobots
        poseRobots = robots
        return robots

    def distance (self, x, y, xa, ya):
        return math.sqrt(((xa-x)**2)+((ya-y)**2))

    def matriceRotation(self, x ,y, theta):
        # [cos(θ) -sin(θ)]   [x]
        # [sin(θ)  cos(θ)] * [y]
        return( cos(theta)*x - sin(theta)*y , sin(theta)*x+cos(theta)*y )
    
    def quaternion_to_yaw(self, x, y, z, w):
        t1 = +2.0 * (w * z + x * y)
        t2 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t1, t2)
        return yaw_z 

    def order_robot(self, ArriveX, ArriveY):

        global poseRobots
        robotIdPose = poseRobots.robots[0].pose # Robot 0

        # 1. récupérer: position et orientation courante du robot + position d'arrivé les deux étant dans le repère du terrain
        robotX = robotIdPose.position.x
        robotY = robotIdPose.position.y
        robotO = robotIdPose.orientation
        robotO = self.quaternion_to_yaw(robotO.x,robotO.y,robotO.z, robotO.w) # Orientation du robot en radian

        # 2. calculer le vecteur allant de la position du robot à la position d'arrivée
        vecteur = (ArriveX-robotX, ArriveY-robotY)
        
        # 3. créer une matrice de rotation anti-horaire correspondant à l'orientation du robot
        # 4. appliquer cette matrice sur le vecteur
        vecteur = self.matriceRotation(vecteur[0],vecteur[1],robotO)

        # Grâce au vecteur on crée une vitesse
        vel_msg = Twist() 
        vel_msg.linear.x= vecteur[0] # Vitesse vers l'avant du robot
        vel_msg.linear.y= vecteur[1] # Vitesse vers la gauche du robot
       
        msg = Order()
        msg.id = 0 # numéro Robot
        msg.velocity = vel_msg # vitesse
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    move_to = MoveTo()

    rclpy.spin_once(move_to)
    global poseRobots
    robotX = poseRobots.robots[0].pose.position.x
    robotY = poseRobots.robots[0].pose.position.y

    path = [(1.0,1.0),(1.0,2.0),(2.0,2.0),(1.5,1.5),(1.0,0.8),(0.0,0.0),(-0.1,0.0)] # Exemple path
    
    for pose in path:
        ArriveX = pose[0]
        ArriveY = pose[1]
        while move_to.distance(robotX, robotY, ArriveX, ArriveY)>0.1: # Tant qu'on est à plus de 10cm de la destination
            rclpy.spin_once(move_to) # récupère la position du robot
            robotX = poseRobots.robots[0].pose.position.x 
            robotY = poseRobots.robots[0].pose.position.y
            move_to.order_robot(ArriveX, ArriveY)

    move_to.destroy_node()
    rclpy.shutdown()
       
if __name__ == '__main__':
    main()
