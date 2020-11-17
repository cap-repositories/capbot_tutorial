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

Esta terminal no se debe cerrar por lo que debe abrir otras terminales para ejecutar otros comandos. **Cuando se ejecuta gazebo desde el launch file, roscore corre internamente, asi que no es necesario llamarlo de nuevo**.

### Rosnode

_Rosnode_ es una función de ROS que permite obtener información de los nodos que están registrados en _rosmaster_. Roscore debe estar corriendo para poder usar rosnode.

Veamos que puede hacer rosnode, ejecuta en una nueva terminal:

```

rosnode list

```

Esto devuelve todos los nodos que se estén corriendo. En nuestro caso solo debería mostrar `/rosout` que es un nodo que siempre se esta ejecutando para manejar la comunicación.

Otro comando es _rosnode info_ el cual responde con la información del nodo solicitado. Esta información debe ser escrita por el desarrollador.

Ejecute:

```

rosnode info /rosout

```

Ahora veremos como activar mas nodos manualmente, pero primero vamos a iniciar la simulación de un robot para analizar la comunicación entre nodos.

Siga los siguientes pasos:

1. En el ROS DE abra la pestaña _Simulations_

2. En la opcion "world" seleccione "Empty + Wall"

3. En la opcion "robot" seleccione "Rosbot"

4. De click en "Start Simulation"

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

Estos muestra que hay varios datos de los cuales nos puede interesar que el dato `ranges` es un arreglo de datos tipo float32, este dato contiene los valores leídos por el sensor desde _angle_min_ a _angle_max aumentando_ lo indicado en _angle_increment_.

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

Esto corresponde a un mensaje del tipo: '[x, y, z]' '[x, y, z]' donde el primer vector es la velocidad y el segundo la rotación.


## Control Manual: enviando mensajes desde la terminal.


