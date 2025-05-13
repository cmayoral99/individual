import rospy
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def move_turtle_45_degrees():
    # Iniciar el nodo
    rospy.init_node('turtle_45_control')

    # Llamar al servicio para crear una tortuga
    rospy.wait_for_service('/spawn')
    spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
    spawn_turtle(5.5, 5.5, 0, 'turtle1')

    # Inicializar el publisher para controlar el movimiento de la tortuga
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Frecuencia de 10 Hz
    
    # Crear el mensaje Twist para controlar el movimiento
    move_cmd = Twist()

    # Establecer velocidad lineal y angular (giro de 45 grados)
    move_cmd.linear.x = 0.0  # No se moverá en línea recta
    move_cmd.angular.z = 0.785  # 45 grados en radianes (0.785 rad)

    # Gira la tortuga 45 grados
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(5):  # Gira durante 5 segundos
        pub.publish(move_cmd)
        rate.sleep()

    # Detener la tortuga después de girar
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        move_turtle_45_degrees()
    except rospy.ROSInterruptException:
        pass
