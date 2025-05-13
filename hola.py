#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, radians, degrees

class TurtleMover:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('turtle_mover', anonymous=True)

        # Subscripción a la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_position)
        # Publicación de los comandos de velocidad
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Frecuencia para mantener el control de la tortuga
        self.update_rate = rospy.Rate(10)  # 10 Hz

        # Variables de posición inicial
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def update_position(self, data):
        """Actualizar la posición y orientación actuales de la tortuga"""
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta

    def get_target_position(self):
        """Solicitar la nueva meta y orientación al usuario"""
        print("\nIntroduce la nueva meta para la tortuga:")
        target_x = float(input("Coordenada x de la meta: "))
        target_y = float(input("Coordenada y de la meta: "))
        target_theta_deg = float(input("Ángulo de orientación deseado (en grados): "))
        return target_x, target_y, radians(target_theta_deg)

    def move_turtle(self, target_x, target_y):
        """Mover la tortuga hacia la meta usando control proporcional"""
        velocity_msg = Twist()
        Kp_distance = 1.5  # Constante proporcional para la distancia
        Kp_angle = 6.0  # Constante proporcional para el ángulo

        while not rospy.is_shutdown():
            # Calcular la distancia a la meta (DTG) y el ángulo hacia la meta (ATG)
            distance_to_goal = sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            angle_to_goal = atan2(target_y - self.current_y, target_x - self.current_x)
            angle_diff = angle_to_goal - self.current_theta

            # Normalización del ángulo para evitar valores fuera de rango
            angle_diff = (angle_diff + 3.14159) % (2 * 3.14159) - 3.14159

            # Velocidades proporcionales a los errores calculados
            velocity_msg.linear.x = Kp_distance * distance_to_goal
            velocity_msg.angular.z = Kp_angle * angle_diff

            self.velocity_publisher.publish(velocity_msg)

            rospy.loginfo("DTG: %.4f | ATG: %.4f°", distance_to_goal, degrees(angle_diff))

            # Detener si estamos suficientemente cerca de la meta
            if distance_to_goal < 0.1:
                break

            self.update_rate.sleep()

        # Detener la tortuga al llegar a la meta
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        rospy.loginfo("Meta alcanzada.\n")

    def rotate_turtle(self, target_theta):
        """Rotar la tortuga hacia el ángulo deseado"""
        velocity_msg = Twist()
        Kp_rotation = 4.0  # Constante proporcional para la rotación

        while not rospy.is_shutdown():
            angle_error = target_theta - self.current_theta
            angle_error = (angle_error + 3.14159) % (2 * 3.14159) - 3.14159

            velocity_msg.angular.z = Kp_rotation * angle_error
            self.velocity_publisher.publish(velocity_msg)

            rospy.loginfo("Error de ángulo: %.4f°", degrees(angle_error))

            # Detener la rotación cuando el error sea pequeño
            if abs(angle_error) < 0.05:
                break

            self.update_rate.sleep()

        # Detener la rotación
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)

    def start(self):
        """Función principal que gestiona el movimiento hacia la meta"""
        while not rospy.is_shutdown():
            target_x, target_y, target_theta = self.get_target_position()
            self.move_turtle(target_x, target_y)
            self.rotate_turtle(target_theta)

if __name__ == '__main__':
    try:
        turtle_controller = TurtleMover()
        turtle_controller.start()
    except rospy.ROSInterruptException:
        pass
