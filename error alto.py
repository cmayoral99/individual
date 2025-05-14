#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees

class TurtleRotationProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_rotacion', anonymous=True)
        
        # Suscripción al topic de la posición de la tortugaAAAAAAAAAA
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
        Kp = 8.0  # Ganancia proporcional aumentada para mayor precisión
        
        # Convertir ángulo objetivo a radianes
        target_theta = radians(target_theta)
        
        while not rospy.is_shutdown():
            # Calcular el error angular
            error_theta = target_theta - self.current_theta
            # Ajustar error al rango [-pi, pi]
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159
            
            # Calcular velocidad angular proporcional al error
            vel_z = Kp * error_theta
            
            # Crear y publicar mensaje Twist
            twist_msg = Twist()
            twist_msg.angular.z = vel_z
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir error en consola
            rospy.loginfo(f"Error de ángulo: {degrees(error_theta):.2f}°")
            
            # Condición de parada con umbral más estricto
            if abs(error_theta) < 0.02:  # umbral reducido (~1.15 grados)
                rospy.loginfo(f"Ángulo objetivo alcanzado: {degrees(target_theta):.2f}°")
                break
            
            self.rate.sleep()
        
        # Detener rotación
        twist_msg.angular.z = 0
        self.velocity_publisher.publish(twist_msg)

        # Esperar 10 segundos antes de regresar a cero
        rospy.loginfo("Esperando 10 segundos antes de regresar a cero grados...")
        rospy.sleep(10)

    def rotate_back_to_zero(self):
        # Regresar a cero grados
        self.rotate_turtle_to_target(0)

    def get_target_angle_from_user(self):
        print("Ingrese el ángulo objetivo de rotación (en grados):")
        return float(input("Ángulo deseado (grados): "))

    def rotate_turtle_interactively(self):
        while not rospy.is_shutdown():
            target_angle = self.get_target_angle_from_user()
            self.rotate_turtle_to_target(target_angle)
            self.rotate_back_to_zero()

if __name__ == '__main__':
    try:
        turtle_rotation_control = TurtleRotationProportionalControl()
        turtle_rotation_control.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
