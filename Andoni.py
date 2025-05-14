#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64
from math import radians, degrees
import matplotlib.pyplot as plt

class TurtleRotationProportionalControl:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('control_tortuga_rotacion', anonymous=True)
        
        # Suscripción al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicador para velocidad angular
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Almacenes para graficar
        self.times = []
        self.angles_deg = []  # Guardaremos el ángulo en grados
        
        # Marca de tiempo inicial
        self.start_time = None

        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Estado interno del ángulo actual (radianes)
        self.current_theta = 0.0

    def pose_callback(self, pose: Pose):
        self.current_theta = pose.theta
        
        # Registra tiempo y ángulo (grados) para graficar
        if self.start_time is not None:
            t = rospy.get_time() - self.start_time
            self.times.append(t)
            self.angles_deg.append(degrees(pose.theta))

        rospy.logdebug(f"Ángulo actual: {degrees(pose.theta):.2f}°")

    def rotate_turtle_to_target(self, target_theta_deg: float):
        Kp = 8.0
        target_theta = radians(target_theta_deg)
        
        # Inicia la marca de tiempo justo antes de iniciar la rotación
        self.start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            error = (target_theta - self.current_theta + 3.14159) % (2 * 3.14159) - 3.14159
            vel_z = Kp * error

            twist = Twist()
            twist.angular.z = vel_z
            self.velocity_publisher.publish(twist)

            rospy.loginfo(f"Error de ángulo: {degrees(error):.2f}°")

            if abs(error) < 0.02:
                rospy.loginfo(f"Ángulo objetivo alcanzado: {target_theta_deg:.2f}°")
                break

            self.rate.sleep()

        # Detener rotación
        self.velocity_publisher.publish(Twist())

        # Espera antes de regresar a cero
        rospy.loginfo("Esperando 10 segundos antes de regresar a cero grados...")
        rospy.sleep(10)

    def rotate_back_to_zero(self):
        self.rotate_turtle_to_target(0)

    def plot_trajectory(self):
        plt.figure()
        plt.plot(self.times, self.angles_deg, label='Ángulo (grados)')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Ángulo (grados)')
        plt.title('Ángulo de rotación de la tortuga vs Tiempo')
        plt.legend()
        plt.grid(True)
        plt.show()

    def get_target_angle_from_user(self) -> float:
        return float(input("Ángulo deseado (grados): "))

    def rotate_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Limpia datos anteriores
            self.times.clear()
            self.angles_deg.clear()

            target = self.get_target_angle_from_user()
            self.rotate_turtle_to_target(target)
            self.rotate_back_to_zero()

            # Muestra gráfica al finalizar la rotación completa
            self.plot_trajectory()

if __name__ == '__main__':
    try:
        controller = TurtleRotationProportionalControl()
        rospy.loginfo("Nodo de control inicializado. Ingrese un ángulo y al terminar verá la gráfica de ángulo vs tiempo.")
        controller.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
