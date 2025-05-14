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
        
        # Publicadores para velocidad angular y posición (x, y)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.x_publisher = rospy.Publisher('/turtle1/x', Float64, queue_size=10)
        self.y_publisher = rospy.Publisher('/turtle1/y', Float64, queue_size=10)
        
        # Almacenes para graficar
        self.times = []
        self.xs = []
        self.ys = []
        
        # Marca de tiempo inicial
        self.start_time = None

        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Estado interno
        self.current_theta = 0.0

    def pose_callback(self, pose: Pose):
        self.current_theta = pose.theta
        
        # Registra tiempo y posición para graficar
        if self.start_time is not None:
            t = rospy.get_time() - self.start_time
            self.times.append(t)
            self.xs.append(pose.x)
            self.ys.append(pose.y)

        # Publica coordenada X e Y para PlotJuggler si se desea
        self.x_publisher.publish(Float64(pose.x))
        self.y_publisher.publish(Float64(pose.y))

        rospy.logdebug(f"Posición publicada -> x: {pose.x:.2f}, y: {pose.y:.2f}")

    def rotate_turtle_to_target(self, target_theta_deg: float):
        Kp = 8.0
        target_theta = radians(target_theta_deg)
        
        while not rospy.is_shutdown():
            error = (target_theta - self.current_theta + 3.14159) % (2 * 3.14159) - 3.14159
            vel_z = Kp * error

            twist = Twist()
            twist.angular.z = vel_z
            self.velocity_publisher.publish(twist)

            rospy.loginfo(f"Error de ángulo: {degrees(error):.2f}°")

            if abs(error) < 0.02:
                rospy.loginfo(f"Ángulo objetivo alcanzado: {degrees(target_theta):.2f}°")
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
        # Crea figura y muestra la trayectoria X(t) e Y(t)
        plt.figure()
        plt.plot(self.times, self.xs, label='X')
        plt.plot(self.times, self.ys, label='Y')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Posición')
        plt.title('Trayectoria de la tortuga vs Tiempo')
        plt.legend()
        plt.grid(True)
        plt.show()

    def get_target_angle_from_user(self) -> float:
        return float(input("Ángulo deseado (grados): "))

    def rotate_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Inicia registro de tiempos
            self.start_time = rospy.get_time()
            # Limpia datos anteriores
            self.times.clear()
            self.xs.clear()
            self.ys.clear()

            target = self.get_target_angle_from_user()
            self.rotate_turtle_to_target(target)
            self.rotate_back_to_zero()

            # Muestra gráfica al finalizar la rotación completa
            self.plot_trajectory()

if __name__ == '__main__':
    try:
        controller = TurtleRotationProportionalControl()
        rospy.loginfo("Nodo de control inicializado. Ingrese un ángulo y al terminar verá la gráfica de posición vs tiempo.")
        controller.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
