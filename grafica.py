#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, degrees
import matplotlib.pyplot as plt

class TurtleRotationProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_rotacion', anonymous=True)
        
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        self.times = []
        self.angles_deg = []
        
        self.start_time = None
        self.current_theta = 0.0
        self.rate = rospy.Rate(10)

    def pose_callback(self, pose: Pose):
        self.current_theta = pose.theta

    def rotate_turtle_to_target(self, target_theta_deg: float):
        Kp = 8.0
        target_theta = radians(target_theta_deg)
        
        self.start_time = rospy.get_time()
        self.times.clear()
        self.angles_deg.clear()
        
        # Rotar hasta alcanzar el objetivo
        while not rospy.is_shutdown():
            error = (target_theta - self.current_theta + 3.14159) % (2 * 3.14159) - 3.14159
            vel_z = Kp * error

            twist = Twist()
            twist.angular.z = vel_z
            self.velocity_publisher.publish(twist)

            t = rospy.get_time() - self.start_time
            self.times.append(t)
            self.angles_deg.append(degrees(self.current_theta))

            if abs(error) < 0.02:
                rospy.loginfo(f"Ángulo objetivo alcanzado: {target_theta_deg:.2f}°")
                break

            self.rate.sleep()
        
        # Mantener posición durante 10 segundos, registrando datos
        hold_start = rospy.get_time()
        while rospy.get_time() - hold_start < 10 and not rospy.is_shutdown():
            t = rospy.get_time() - self.start_time
            self.times.append(t)
            self.angles_deg.append(degrees(self.current_theta))
            self.velocity_publisher.publish(Twist())  # Detener rotación
            self.rate.sleep()
        
        self.velocity_publisher.publish(Twist())  # Asegurar parada

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
            target = self.get_target_angle_from_user()
            self.rotate_turtle_to_target(target)
            self.plot_trajectory()

if __name__ == '__main__':
    try:
        controller = TurtleRotationProportionalControl()
        rospy.loginfo("Nodo de control inicializado. Ingrese un ángulo y verá la gráfica de ángulo vs tiempo.")
        controller.rotate_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
