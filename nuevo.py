#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees

class TurtleMover:
    def __init__(self):
        # Inicializa el nodo de ROSiiii
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
        self.step_angle = 5.625  # Ángulo por paso del motor (en grados)
        self.target_angle = 45  # Ángulo deseado en grados
        self.steps_needed = int(self.target_angle / self.step_angle)  # Número de pasos (8 pasos)
        self.steps_taken = 0  # Número de pasos dados

    def update_position(self, data):
        """Actualizar la posición y orientación actuales de la tortuga"""
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta

    def rotate_turtle(self):
        """Simular el movimiento en pasos de 5.625 grados"""
        velocity_msg = Twist()
        Kp_rotation = 4.0  # Constante proporcional para la rotación
        total_rotation = 0  # Acumulamos el total de rotación de la tortuga

        while self.steps_taken < self.steps_needed:
            # Rotar en incrementos de 5.625 grados
            velocity_msg.angular.z = Kp_rotation * self.step_angle
            self.velocity_publisher.publish(velocity_msg)
            
            rospy.loginfo(f"Paso {self.steps_taken + 1}: Girando {self.step_angle} grados.")
            
            # Esperamos un poco entre cada paso (ajustamos el tiempo)
            rospy.sleep(1)  # Pausar por un segundo entre cada paso

            # Actualizar la rotación total
            total_rotation += self.step_angle
            self.steps_taken += 1

            # Verificar si alcanzamos el ángulo objetivo
            if self.steps_taken == self.steps_needed:
                rospy.loginfo(f"Se alcanzaron los {total_rotation} grados.")
                break

        # Detener la tortuga
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        rospy.loginfo("Giro completado.")

    def start(self):
        """Función principal para controlar el movimiento hacia la meta"""
        self.rotate_turtle()

if __name__ == '__main__':
    try:
        turtle_controller = TurtleMover()
        turtle_controller.start()
    except rospy.ROSInterruptException:
        pass
