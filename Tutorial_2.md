# Tutorial 2: Controlando el movimiento del robot
En este tutorial utilizaremos diferentes estrategias para controlar manual y automaticamente el movimiento de robot. primero se presentan los conceptos fundamentales de la comunicacion entre nodos de ROS.


## Navegar por el sistema de archivos ROS

Los paquetes son la unidad de organización principales de ROS. Cada paquete puede contener bibliotecas, ejecutables, scripts u otros elementos. Los paquetes están asociados a un manifiesto (package.xml) el cual contiene descripción del paquete y sirve para definir dependencias entre paquetes y para capturar meta información sobre el paquete como versión, mantenedor, licencia, etc.

En un proyecto complejo se pueden usar muchos paquetes lo que hace que navegar usando comandos de consola como `cd` o `ls` sea muy difícil. Por esta razón, **ROS** tiene su propio sistema de navegación basado en comandos pero enfocado a encontrar paquetes sin tener que conocer o especificar la ruta.

Intente los siguientes comando:

```
$ rospack find rospy

$ roscd roscpp

$ rosls rospy
```

- El primer comando `rospack find` devuelve la ruta del paquete solo con su nombre.

- El comando `roscd` permite entrar al directorio (carpeta) del paquete solo por su nombre. También funciona con subcarpetas.

- El comando `rosls` devuelve la lista de carpetas y archivos que contiene un paquete.

Los paquete _roscpp_ y _rospy_ permiten correr funciones de **ROS** desde C++ y python respectivamente; se verán mas adelante.

Cierre la terminal y abra una nueva para el siguiente paso.

## Autocompletado por medio de TAP

Una función muy útil de **ROS** para hacer mas rápido el uso de la terminal de comandos, es el autocompletado por medio de la tecla Tab. Con esto, solo es necesario escribir las primeras letras de cada nombre y presionar Tab para que se complete el resto o se muestren las opciones cuando hay mas de una.

En la nueva terminal escriba los siguiente pero presionando Tab **dos** veces al final.

```

$ rosls rosc

```

Vera que se muestran todos los paquetes que empiezan por `rosc`, ahora escriba lo siguiente y presiones una vez Tab.

```

$ rosls roscpp_s

```

Vera que se completa `rosls roscpp_serialization` que es la única opción para completar. (puede ser otra dependiendo de los paquetes instalados).

De esta forma se pueden completar complicadas rutas de archivos para ejecutar los comandos correspondiente.

## Nodos

Los nodos son las unidades basicas de ROS, la arquitectura de ROS consiste en nodos que se comunican entre ellos, los paquetes son agrupaciones de nodos.

Conceptos básicos de la comunicación en ROS:

- **Nodos (node):** un nodo es un ejecutable que utiliza ROS para comunicarse con otros nodos.

- **Mensajes (messages):** Tipo de datos en ROS utilizado al suscribirse o publicar en un tema.

- **Temas (topic):** Los nodos pueden publicar mensajes en un tema, así como suscribirse a un tema para recibir mensajes.

- **Maestro (master):** Servicio encargado del registro de nombres en ROS (es decir, ayuda a los nodos a encontrarse)

- **rosout**: Equivalente ROS de stdout / stderr (funciones de terminal de linux)

- **roscore:** Master + rosout + servidor de parámetros (el servidor de parámetros se presentará más adelante)

Un nodo es un archivo ejecutable dentro de un paquete ROS. Los nodos ROS utilizan _rospy_ o _roscpp_ para comunicarse con otros nodos. Los nodos pueden publicar o suscribirse a un tema. Los nodos también pueden proporcionar o usar un Servicio.

### Roscore

Para utilizar la infraestructura de comunicación de ROS es necesario que _roscore_ este ejecutándose, esto implica tener una terminal dedicada donde se ejecuta el siguiente comando:

```

$ roscore

```

Esta terminal no se debe cerrar por lo que debe abrir otras terminales para ejecutar otros comandos. **Cuando se ejecuta gazebo desde el launch file, roscore corre internamente, asi que no es necesario llamarlo de nuevo mientras gazebo este activo**.

### Rosnode

_Rosnode_ es una función de ROS que permite obtener información de los nodos que están registrados en _rosmaster_. Roscore debe estar corriendo para poder usar rosnode.

Veamos que puede hacer rosnode, ejecuta en una nueva terminal:

```
rosnode list
```

Esto devuelve todos los nodos que se estén corriendo. En nuestro caso solo debería mostrar `/rosout` que es un nodo que siempre se esta ejecutando para manejar la comunicación. (si sale error en comunicacion es porque no estas ejecutando roscore en otra terminal).

Otro comando es _rosnode info_ el cual responde con la información del nodo solicitado. Esta información debe ser escrita por el desarrollador.

Ejecute:

```
rosnode info /rosout
```

Ahora veremos como activar mas nodos manualmente, pero primero vamos a iniciar la simulación de un robot para analizar la comunicación entre nodos.

Cierre todas las terminales y siga los siguientes pasos:

1. En la carpeta capbot_ws ejecute ```catkin_make``` para compilar todos los nodos y ejecute ```source devel/setup.bash```.

2. Ejecute gazebo con ```roslaunch capbot_gazebo capbot.launch```

3. Ejecute el tf_broadcaster con ```rosrun capbot_setup_tf tf_broadcaster```


La simulación puede tardar varios segundos o minutos en iniciar. Una vez cargado todo el ambiente de simulación, ejecute nuevamente `rosnode list` para ver todos los nodos que ahora se están ejecutando.

Aparecen los siguientes nodos.

```

/gazebo

/gmapping_node

/joint_state_controller_spawner

/move_base

/robot_state_publisher

/rosout

/rviz

```

Estos nodos iniciaron automáticamente, pero otros nodos se pueden iniciar desde la terminal usando `rosrun`, veremos mas adelante como usar este comando con los nodos que creamos.

## Comunicación por medio de topics

ROS permite dos tipos de comunicaciones entre nodos, _Topics_ y _Services_, vamos a enfocarnos primero en los topics, los servicios los estudiaremos a continuación.

Los _Topics_ permiten una comunicación tipo PUB/SUB (publicación - suscripción) en la cual un nodo crea un tema (topic) y otros nodos se suscriben a el, cada que se publique un mensaje en el topic, todos los nodos suscritos pueden leerlo.

El comando base para trabajar con _Topics_ es **rostopic**.
Un primer uso de este comando es la función de ayuda para leer otras funciones y comandos, en una nueva terminal escriba lo siguiente:

```

$ rostopic -h

```

Veremos con detalle algunos de estos comandos.

### rostopic echo

`rostopic echo` permite ver en la terminal los mensajes publicados en un tema.

Para conocer los temas que están abiertos podemos escribir el siguiente comando:

```

$ rostopic list

```

Esto despliega una lista de los temas abiertos, ahora vamos a leer el tema _/scan_ el cual es usado por el robot para publicar constantemente los valores leídos por el sensor laser de 360 grados.

```

$ rostopic echo /scan

```

No te alarmes!, aparecerán muchas lineas de información, es normal solo tienes que presionar _Ctrl + Z_ para dejar de leer.

Esto nos muestra un ejemplo de un mensaje publicado constantemente en un tema. Para usar esa información es necesario crear una suscripción, eso lo haremos mas adelante.

### Tipos de mensaje

Los temas transmiten mensajes, un mensaje tiene una estructura de datos compleja que puede ir desde un valor único hasta un conjunto de datos de diferentes tipos incluidos arreglos y listas.

Es necesario conocer el tipo de mensaje asociado a un tema para poder usarlo, ya sea para interpretar los datos y usarlos en una aplicación o para poder enviar un mensaje con la estructura correcta.

Para conocer el tipo de mensaje asociado a un tema, se usa el comando `rostopic type [topic]`. Podemos usarlo con el tema `/scan` así:

```

$ rostopic type /scan

# sensor_msgs/LaserScan

```

Ahora podemos usar `rosmsg show [message topic]` para conocer la estructura detallada del mensaje así:

```

$ rosmsg show sensor_msgs/LaserScan

```

Respuesta:

```
std_msgs/Header header
uint32 seq
time stamp
string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

Estos muestra que hay varios datos de los cuales nos puede interesar que el dato `ranges` que es un arreglo de datos tipo float32, este dato contiene los valores leídos por el sensor desde _angle_min_ a _angle_max_ aumentando lo indicado en _angle_increment_.

Como otro ejemplo, podemos revisar la estructura del tema _/cmd_vel_, este tema es usado para enviar mensajes de movimiento al robot.

```
$ rostopic type /cmd_vel
```

Respuesta:

```
geometry_msgs/Twist
```

Ahora buscamos la estructura del mensaje:

```
$ rosmsg show geometry_msgs/Twist
```

Respuesta:

```
geometry_msgs/Vector3 linear
float64 x
float64 y
float64 z
geometry_msgs/Vector3 angular
float64 x
float64 y
float64 z
```

Esto corresponde a un mensaje del tipo: '[x, y, z]' '[x, y, z]' donde el primer vector es la velocidad lineal y el segundo la velocidad de rotación.


## Control Manual: enviando mensajes desde la terminal.

el comando `rostopic pub` permite escribir un mensaje en un tema desde la terminal.

La sintaxis del comando es:

```

rostopic pub [topic] [msg_type] [args]

```

Donde,

- [topic] es el nombre del tema tal como aparece al usar `rostopic list`

- [msg_type] es el tipo de mensaje tal como aparece al usar `rostopic type`

- [args] es el contenido del mesaje que debe tener la misma estructura mostra al usar `rosmsg show`

Ahora usemos este comando para mover el robot. en la terminal escribe:

```

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.6, 0.0, 0.0]' '[0.0, 0.0, 2.0]'

```

El atributo `-1` indica que envía un solo mensaje y termina la comunicación.

En el simulador podrá ver como el robot inicia su movimiento donde '[0.6, 0.0, 0.0]' indica una velocidad lineal en X y '[0.0, 0.0, 2.0]' una velocidad de rotación en Z. Ahora tome un tiempo para jugar un poco con el robot cambiando estos valores manualmente.

## Control Manual: usar un keyboard teleop
Una de las ventajas de ROS es poder usar nodos y paquetes que ya estan desarrollados e integrarlos facilmente en mi proyecto.
Para este ejemplo, usaremos el node teleop_twist_keyboard que nos permite mover el robot desde el teclado del computador.

Primero en una nueva terminal instalamos la libreria:
```
$ sudo apt-get install ros-noetic-teleop-twist-keyboard
```

A continuacion, ejecutamos el nodo
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

En la terminal aparecen instrucciones de como usar las teclas para mover el robot.

Esta libreria tambien se puee modificar descargando el codigo fuente desde github y cambiando por ejemplo el nombre del topic en el que se publica el mansaje de movimiento, esto resulta util si se tienen varios robots en la simulacion y cada unos "escucha" en topics diferentes.

# Control automatico: crear un nodo que controle en funcion del sensor

para crear un nodo que controle el robot de forma automatica, primero debemos crear un paquete que contenga este nodo. para eso creamos el paquete capbot_driver con la dependencia rospy:
```
$ cd capbot_ws/src
$ catkin_create_pkg capbot_driver rospy
```
Ahora entramos a la carpeta capbot_driver y creamos la carpeta scripts. En esta carpeta se guardan los achivos de python.
```
$ cd capbot_drive
$ mkdir scripts
```

En esta carpeta debemos crear el archivo driver.py con el siguiente contenido.
```
#!/usr/bin/env python

import rospy #todo nodo de ROS en python debe importar rospy

from sensor_msgs.msg import LaserScan #importa los tipos de mensaje

from geometry_msgs.msg import Twist

# le indicamos a rospy el nombre del nodo.
rospy.init_node('evadir1') #con este nombre se registra en el rosmaster
#Creamos un objeto para publicar en el topic /cmd_vel, Twist es el tipo de mensaje
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#rospy.rate() ndica la frecuencia en Hz con la que se va a repetir el loop de control.
rate = rospy.Rate(2)
#crea un objeto para guardar el mensaje de tipo Twist
vel = Twist()
#Se da un valor inicial a la velocidad del robot y la distancia del sensor
vel.linear.x = 0.3
vel.angular.z = 0
d = 0.1
#publica a velocidad (se envia al robot)
pub.publish(vel)
#creamos la funcion que haga girar el robot cuando detecte un objeto
def turn(msg): #el unico argumento es el arreglo de lecturas del laser
d = msg.ranges[0] #optiene la lectura de laser en la primera posicion del arreglo
# d es la distancia detectada por el laser junto en frente del robot en m
if d < 0.7: #se hay un bjeto muy cerca, gira
vel.angular.z = 0.5
vel.linear.x = 0
else: #si no, avanza
vel.linear.x = 0.3
vel.angular.z = 0
pub.publish(vel) #actualiza la velocidad en el robot

"""
se crea una suscripcion al topic /scan (medidor laser de 360 grados.
el segundo argumento es el tipo de mensaje LaserScan (se importo)
el tercer argumento es la funcion que atiende el mensaje recibido "callback"
"""

rospy.Subscriber("/scan", LaserScan, turn)

#inicia la ejecucion periodica cada 0.5s segun se indique en rospy.rate
rospy.spin()
```
Tome un tiempo para entender el funcionamiento del script leyendo los comentarios.

Guardamos el archivo y lo volvemos ejecutable con el siguiente comando:
```
$ chmod -x driver.py
```
Ahora debemos agregar el archivo de python a el CMakeLists.txt de la carpeta capbot_driver agregan el siguiente codigo al final del archivo.
```
catkin_install_python(PROGRAMS
   scripts/micontrol.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 ```
 
La funcion catkin_install_python esta por defecto comentada en el CMakeLists, es una buena practica descomentar la funcion y agregar los atributos correspondientes para que el archivo quede mas ordenado.
 
 Finalmente debemos compilar (asi python no se compile) con catkin_make y ejecutar el nodo.
 ```
 $ cd ~/capbot_ws
 $ catkin_make
 $ source devel/setup.bash
 $ rosrun capbot_driver mydriver.py
 ```
 Podra ver en el simulador (si todo sale bien) que el robot se mueve y gira cuando detecta algun obstaculo.
 
 ## Actividad 1

Modifique el archivo mydriver.py para que el robot mejore la evasión de obstáculos utilizando un rango mayor del LaserScan y haciendo que gire mas rápido cuanto mas cerca este del obstáculo.
 
 
 



