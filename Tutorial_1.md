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

primero nos aseguramos de tener instaladas las dependencias para controlar gazebo desde ros
```
$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
Crear el nuevo paquete con las dependencias necesarias
```
$ cd ~/capbot_ws/src
$ catkin_create_pkg capbot_gazebo gazebo_ros roscpp gazebo_msgs gazebo_plugins gazebo_ros_control
```
Crear las carpetas estandar para el almacenamiento de los archivos de la simulacion.

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
se puede notar que el argumento "world_name" hace referencia a un archivo **capbot.world** que no a sido creado y el nodo "spawn_urdf" hace referencia al archivo **capbot.urdf** creado en el punto anterior.

Para crear el entorno tipo ***world*** para la simulacion, se debe abrir gazebo y agregar los elementos que correspondan, finalizar guardando el modelo como capbot.world en la carpeta correspondiente. para abrir gazebo solo se debe escribir ***gazebo*** en la consola y explorar las funciones de agregar objetos y crear geometrias para poblar la simulacion.

Para iniciar la simulacion desde el archivo capbot.launch se utiliza la funcion roslaunch asi:
```
$ source devel/setup.bash
$ roslaunch capbot_gazebo capbot.launch
```
nota: el comando source devel/setup.bash debe usarse cada que se crean nuevos paquetes justo despues e usar catkin make para que los nuevos paquetes puedan usarse desde la terminal.
Este comando abrirá gazebo cargando el entorno y el robot pero no permitira mover el robot desde ROS (todavia), de echo, no es nada diferente al archivo ***gazebo.launch*** que usamos anteriormente, pero iremos agregando mas funciones.

## Configurar el robot para ser simulado en gazebo
El modelo del robot que es interpretado por gazebo es el archivo capbot.urdf, en este archivo se debera incluir toda la informacion y funciones (que llamaremos plugins) que permiten a gazebo interactuar con el robot para producir el movimiento o agregar sensores (simulados).

primero entendamos que contiene un archivo de descripcion de robot tipo URDF.

### Archivos URDF
URDF es un lenguaje de representacion de robot, algo similar a una matriz DH pero con una jerarquia, propiedades fisicas y modelos 3D, es decir, permite representar la cinematica, dinamica y geometria del robot. La forma de representar toda esta informacion es por medio de un archivo XML, asi que para entender urdf se debe entender algo de xml.

EL elemento principal del archivo urdf es el elemento robot con el unico atributo name, el cual contiene  pricipalmente elementos link y joint que se relacionan para formar el robot.
```
<robot name="robotname">
  <link> ... </link>
  <link> ... </link>
  <link> ... </link>
  <joint> .... </joint>
  <joint> .... </joint>
  <joint> .... </joint>
</robot>
```
adicionalmente, el elemento robot puede contener elementos como los plugins de gazebo para complementar el modelo.

los elementos tipo link describen los eslabones del robot, para un brazo robotico serian los eslabones que componen la cadena cinematica, para un robot movil seran las ruedas y el cuerpo del robot. Los elementos link tienen un atributo name por el que se le identifica, los subelementos que describen los link son:
*inertial: difine las propiedades inerciales del eslabon, contiene los siguientes elementos:
** origin: posicion y rotacion del origen del eslabon
** mass: valor de la masa en kg
** inertia: momento de inercia con respecto al origen
*visual: propiedades visuales del eslabon, contiene los siguientes elementos:
** origin: origen visual del elemento
** geometry: describe el modelo 3D del elemento, nomalmente contiene un elementos mesh que hace referencia a un archivo STL (modelo 3D).
** material: propieades visuales del eslabon como el color en formato rgba
* collision: propiedades para detectar colisiones a partir del espacio ocupado por el eslabon, normalmente se copian las mismas propiedades origin y geometry de elemento visual.



*colores
*diffretential drive
*sensor

#Navegacion 2D en ROS

El Navigation Stak es un componente de ros que permite a un robot movil ser dirigido a un obetivo de forma autonoma, evadiendo obstaculos y definiendo continuamente una trayectoria. Para usar el paquete de navegacion se debe previamente tener las siguientes funcionalidades configuradas:

*El robot debe publicar informacion de las coordendas del marco de los marcos de referencia usando mensajes de tipo ***tf***.
*El robot debe publicar informacion del sensor Lidar o una camara de profundidad en los mensajes ***sensor_msgs/LaserScan*** o ***sensor_msgs/PointCloud*** respectivamente.
*Un nodo debe estar publicando informacion de la odometria usando los mensaes ***tf*** y ***nav_msgs/Odometry***.
*EL robot debe moverse al recibir mensajes de tipo ***geometry_msgs/Twist***.
 

