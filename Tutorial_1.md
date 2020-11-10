# capbot_tutorial

## Configurar el Workspace 

```
$ mkdir capbot_ws
$ cd capbot_ws
$ mkdir src
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
```
##Preparar el Modelo del robot

Crear paquete con dependencia de urdf (para visualizar el modelo desde ros)
```
$ cd src
$ catkin_create_pkg capbot_description urdf
$ cd capbot_description
```
Dentro de la carpeta ***capbot_description*** que se acaba de crear, se deben poner las carpetas ***urdf*** y ***meshes*** que se pueden descargar[aqui](https://javerianacaliedu-my.sharepoint.com/:f:/g/personal/juandavid_contreras_javerianacali_edu_co/EmJIYJQKr6xPlEttRKYFlH4ByxAgbMaU-C1fkgPEM6wkOA?e=o5mWnd). En la carpeta urdf esta el archivo capbot.urdf con la descripcion de la conematica y dinamica del robot. En la carpeta meshes estan los archivos STL de los modelos 3D que componen el robot, estos archivos se ensamblan segun la cinematica descrita en  capbot.urdf.

A continuacion se recomiando hacer el analisis del modelo urdf utilizando un ***parser***. Para esto cree una carpeta **src/** dentro de capbot_description y en un editor de texto cree un archivo llamado **parser.cpp** y guardelo en la nueva carpeta **src/**. El contenido de **parser.cpp** es el siguiente:

```
#include <urdf/model.h>
#include "ros/ros.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  return 0;
}
```
Para ejecutar el codigo, primero agregue las siguientes líneas al archivo CMakeList.txt de la carpeta capbot_description:
```
 add_executable(parser src/parser.cpp)
 target_link_libraries(parser ${catkin_LIBRARIES})
 ```

lo siguiente sera Construir (build) el paquete y ejecútelo, para esto en la terminal vaya a la carpeta capbot_ws usando cd o cd .. y pegue los siguientes comandos.
```
$ catkin_make
$ ./devel/lib/capbot_description/parser ./src/capbot_description/urdf/capbot.urdf
```
Si el analisis fue correcto se presentara un mensaje "Successfully parsed urdf file", de lo contrario se mostrara el error detectado.

Para visualizar el modelo URDF, podemos usar gazebo o rviz, lo recomendable es crear un archivo de lanzamiento "launch file".

Para crear nuestro primer launch file debemos crear una carpeta llamada launch en capbot_description y crear un archivos llamado gazebo.launch con el siguiente contenido:
```
<launch>
  <include
    file="$(find gazebo_ros)/launch/empty_world.launch" />
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    type="spawn_model"
    args="-file $(find capbot_description)/urdf/capbot.urdf -urdf -model capbot"
    output="screen" />
</launch>
```
Recuerda guardar el documento.

los launch files son una herramienta muy util para iniciar varios nodos o funciones de ROS de forma integrada. en este archivo estamos abriendo un archivos de gazebo y entregando como argumento nuestro modelo capbot.urdf.

Para ejecutar el launch file debemos ejecutar los siguientes comandos.
```
$ cd ~/capbot_ws
$ source devel/setup.bash
$ roslaunch capbot_description gazebo.launch
```
Debera abrir el simulador Gazebo con el modelo del robot. Todavia no podemos mover el robot debido a que no tenemos ningun nodo que controle la velocidad de las ruedas.
___

## Crear un paquete de Gazebo para ROS

primero aseurarse de tener instaladas las dependencias para controlar gazebo desde ros
```
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
Crear el nuevo paquete
```
$ cd ~/capbot_ws/src
$ catkin_create_pkg capbot_gazebo gazebo_ros roscpp gazebo_msgs gazebo_plugins gazebo_ros_control
```
Crear las carpetas estandar para el almacenamiento de los archivos de la simulacion

```
$ makdir launch
$ mkdir materials
$ mkdir models
$ mkdir worlds
$ mkdir src
```

Crear el archivo **capbot.launch** en la carpeta launch con el siguiente contenido:
```
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find capbot_gazebo)/worlds/capbot.world"/>
  </include>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find capbot_description)/urdf/capbot.urdf -urdf -z 1 -model capbot" />
</launch>
```
Este archivo permite lanzar la simulacion en gazebo cargando automaticamente el entorno y el robot.
se puede notar que el argumento "world_name" hace referencia a un archivo **capbot.world**que no a sido creado y el nodo "spawn_urdf" hace referencia al archivo **capbot.urdf** creado en el punto anterior.

Para crear el entorno tipo ***world*** para la simulacion, se debe abrir gazebo y agregar los elementos que correspondan, finalizar guardando el modelo como capbot.world en la carpeta correspondiente.

Para iniciar la simulacion desde el archivo capbot.launch se utiliza la funcion roslaunch asi:
```
roslaunch capbot_gazebo capbot.launch
```
Este comando abrirá gazebo cargando el entorno y el robot pero no permitira mover el robot desde ROS.

## Configurar el robot para ser simulado en gazebo
El modelo del robot que es interpretado por gazebo es el archivo capbot.urdf, en este archivo se debe incluir toda la informacion y funciones (que llamaremos plugins) que permiten a gazebo interactuar con el robot para producir el movimiento o agregar sensores (simulados).

*colores
*diffretential drive
*sensor

#Navegacion 2D en ROS

El Navigation Stak es un componente de ros que permite a un robot movil ser dirigido a un obetivo de forma autonoma, evadiendo obstaculos y definiendo continuamente una trayectoria. Para usar el paquete de navegacion se debe previamente tener las siguientes funcionalidades configuradas:

*El robot debe publicar informacion de las coordendas del marco de los marcos de referencia usando mensajes de tipo ***tf***.
*El robot debe publicar informacion del sensor Lidar o una camara de profundidad en los mensajes ***sensor_msgs/LaserScan*** o ***sensor_msgs/PointCloud*** respectivamente.
*Un nodo debe estar publicando informacion de la odometria usando los mensaes ***tf*** y ***nav_msgs/Odometry***.
*EL robot debe moverse al recibir mensajes de tipo ***geometry_msgs/Twist***.
 
