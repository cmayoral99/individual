#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees

class TurtleRotationProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_rotacion', anonymous=True)
        
        # Suscripción al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicación en el topic de los comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_theta = 0  # Ángulo actual de la tortuga

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_theta = pose.theta

    def rotate_turtle_to_target(self, target_theta):
        # Constante de proporcionalidad del controlador (ajustable)
        Kp = 5.0  # Aumentamos Kp para hacer la rotación más rápida
        
        # El ángulo objetivo es el que el usuario ingresa (convertido a radianes)
        target_theta = radians(target_theta)
        
        # Variable para calcular el error de ángulo
        error_theta = target_theta - self.current_theta
        
        while not rospy.is_shutdown():
            # Calcular el error de ángulo
            error_theta = target_theta - self.current_theta

            # Asegurar que el error esté en el rango [-pi, pi] para una rotación más precisa
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

            # Calcular la velocidad angular (proporcional al error)
            vel_z = Kp * error_theta

            # Crear el mensaje Twist para mover la tortuga
            twist_msg = Twist()
            twist_msg.angular.z = vel_z
            
            # Publicar el comando de rotación
            self.velocity_publisher.publish(twist_msg)

            # Imprimir la información en la consola
            rospy.loginfo(f"Error de ángulo: {degrees(error_theta):.2f}°")
            
            # Verificar si hemos alcanzado el ángulo objetivo (error pequeño)
            if abs(error_theta) < 0.1:  # Se puede ajustar el umbral de error
                rospy.loginfo(f"Ángulo objetivo alcanzado: {degrees(target_theta):.2f}°")
                break

            # Esperar hasta la siguiente iteración
            self.rate.sleep()

        # Detener la rotación
        twist_msg.angular.z = 0
        self.velocity_publisher.publish(twist_msg)

        # Esperar 10 segundos antes de comenzar el regreso
        rospy.loginfo("Esperando 10 segundos antes de regresar a cero grados...")
        rospy.sleep(10)  # Esperar 10 segundos

    def rotate_back_to_zero(self):
        """Hacer que la tortuga regrese a cero grados"""
        target_theta = 0  # El ángulo objetivo es cero grados
        
        self.rotate_turtle_to_target(target_theta)  # Llamar a la función para mover la tortuga a cero grados

    def get_target_angle_from_user(self):
        print("Ingrese el ángulo objetivo de rotación (en grados):")
        return float(input("Ángulo deseado (grados): "))

    def rotate_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener el ángulo objetivo del usuario
            target_angle = self.get_target_angle_from_user()

            # Mover la tortuga al ángulo objetivo
            self.rotate_turtle_to_target(target_angle)

            # Después de alcanzar el ángulo objetivo, regresar a cero grados
            self.rotate_back_to_zero()

if __name__ == '__main__':
    try:
        turtle_rotation_control = TurtleRotationProportionalControl()
        turtle_rotation_control.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
