#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees

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
        self.encoder_steps = 0  # Contador de pasos del motor (simulando encoder)

    def update_position(self, data):
        """Actualizar la posición y orientación actuales de la tortuga"""
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta

    def rotate_turtle(self, target_theta):
        """Rotar la tortuga hacia el ángulo deseado"""
        velocity_msg = Twist()
        Kp_rotation = 4.0  # Constante proporcional para la rotación

        while not rospy.is_shutdown():
            # Calcular el error de ángulo
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

    def simulate_button_press(self):
        """Esperar que el usuario presione una tecla para girar 45 grados"""
        while not rospy.is_shutdown():
            input("Presiona Enter para mover la tortuga 45 grados...")  # Simula un botón de presión
            self.encoder_steps += 1  # Incrementar los pasos simulando el encoder

            print(f"Simulación de encoder - Pasos: {self.encoder_steps}")
            self.rotate_turtle(radians(45))  # Hacer que la tortuga rote 45 grados

            # Después de girar, espera una nueva tecla para el siguiente movimiento
            rospy.loginfo("Tortuga ha girado 45 grados. Esperando la siguiente pulsación...\n")

    def start(self):
        """Función principal que gestiona el movimiento hacia la meta"""
        self.simulate_button_press()

if __name__ == '__main__':
    try:
        turtle_controller = TurtleMover()
        turtle_controller.start()
    except rospy.ROSInterruptException:
        pass
