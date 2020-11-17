# Navegacion 2D en ROS

El Navigation Stak es un componente de ros que permite a un robot movil ser dirigido a un obetivo de forma autonoma, evadiendo obstaculos y definiendo continuamente una trayectoria. Para usar el paquete de navegacion se debe previamente tener las siguientes funcionalidades configuradas:

*El robot debe publicar informacion de las coordendas del marco de los marcos de referencia usando mensajes de tipo ***tf***.
*El robot debe publicar informacion del sensor Lidar o una camara de profundidad en los mensajes ***sensor_msgs/LaserScan*** o ***sensor_msgs/PointCloud*** respectivamente.
*Un nodo debe estar publicando informacion de la odometria usando los mensaes ***tf*** y ***nav_msgs/Odometry***.
*EL robot debe moverse al recibir mensajes de tipo ***geometry_msgs/Twist***.
