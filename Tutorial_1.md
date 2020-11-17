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
$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
Crear el nuevo paquete con las dependencias necesarias
```
$ cd ~/capbot_ws/src
$ catkin_create_pkg capbot_gazebo gazebo_ros roscpp gazebo_msgs gazebo_plugins gazebo_ros_control
```
Crear las carpetas estandar para el almacenamiento de los archivos de la simulacion.

```
$ cd capbot_gazebo
$ mkdir launch
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
* inertial: difine las propiedades inerciales del eslabon, contiene los siguientes elementos:
  - origin: posicion y rotacion del origen del eslabon
  - mass: valor de la masa en kg
  - inertia: momento de inercia con respecto al origen
* visual: propiedades visuales del eslabon, contiene los siguientes elementos:
  - origin: origen visual del elemento
  - geometry: describe el modelo 3D del elemento, nomalmente contiene un elementos mesh que hace referencia a un archivo STL (modelo 3D).
  - material: propieades visuales del eslabon como el color en formato rgba
* collision: propiedades para detectar colisiones a partir del espacio ocupado por el eslabon, normalmente se copian las mismas propiedades origin y geometry de elemento visual.
```
<link
    name="base_link">
    <inertial>
      <origin
        xyz="0.019462 0.00056937 0.12444"
        rpy="0 0 0" />
      <mass
        value="8.127" />
      <inertia
        ixx="0.083537"
        ixy="-9.6838E-07"
        ixz="-5.7145E-06"
        iyy="0.11259"
        iyz="2.8774E-06"
        izz="0.1919" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://capbot_description/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://capbot_description/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
```
El elementos Joint define la articulaciones que uno dos eslabones. Este elemento tiene dos atributos:
* name: en nombre por el que se le identifica
* type: el tipo de articulacion que puede ser:
- revolute: una articulación de bisagra que gira a lo largo del eje y tiene un rango limitado especificado por los límites superior e inferior.
- continuous: una articulación en bisagra continua que gira en torno al eje y no tiene límites superior e inferior.
- prismatic: una junta deslizante que se mueve a lo largo del eje, y tiene un rango limitado especificado por los límites superior e inferior.
- fixed: Esto no es realmente una articulación, ya que no puede moverse. Todos los grados de libertad están bloqueados. Este tipo de unión no requiere ejes.
- floating: Esta articulación permite el movimiento de los 6 grados de libertad.
- planar: Esta articulación permite el movimiento en un plano perpendicular al eje.

Adicionalmente, el elemento joint contiene otros elementos que describen el movimiento y jerarquia:
* origin: posicion y rotacion de la articulacion con respecto al origin del eslabon padre
* parent: hace referencia al eslabon que precede a la articulacion, o el eslabon al que esta vinculado. esta referencia se hace en el atributo link dando en nombre del eslabon.
* child: hace referencia al eslabon que se movera por la articulacion. esta referencia se hace en el atributo link dando en nombre del eslabon.
* axis: difine por medio del atributo xyz cual es el eje sobre el que se hace el movimiento.
```
<joint
    name="L_joint"
    type="continuous">
    <origin
      xyz="0 0.238 0"
      rpy="1.5708 0 3.1416" />
    <parent
      link="base_link" />
    <child
      link="L_Link" />
    <axis
      xyz="0 0 1" />
  </joint>
```
el modelo urdf del capbot fue creado de forma automatica utilizando el complemento urdf para solidworks.

## gazebo plugins
Para conectar el modelo urdf de forma dinamica con gazebo, se deben incluir los plugins de gazebo en el archivo urdf, esto es simplemente agregar otros elementos.

Un listado completo de los plugins se puede encontrar en este [link](http://gazebosim.org/tutorials?tut=ros_gzplugins).


El primer plugin que debemos agregar al modelo de nuestro robot es el ***differential_drive_controller***, el cual nos permite controlar el movimiento del capbot. lo que hace este plugin es crear dentro de gazebo un controlador para robots diferenciales (como el capbot), este controlador recibe mensajes tipo Twist y los convierte en las correspondientes velocidades de rotacion en las ruedas. Para hacer esta conversion, el plugin necesita conocer la gemetria del robot (diamtro de las ruedas, distancia entre estas), tambien conocer propiedades cinematicas y dinamica como la aceletacion y el torque de las ruedas para representar de forma realista los motores. Finalmente, el plugin requiere informacion para comunicarse con ROS como el nombre de los topics y que informacion debe publicar o no. 

Todas las propiedades anteriores se agregan pegando el siguiente codigo dentro de elementos robot del archivo urdf:
```
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- Taza de actualizacion en Hz -->
    <updateRate>20</updateRate>
    <!-- nombre de la articulacion izquierda -->
    <leftJoint>L_joint</leftJoint>
    <!-- nombre de la articulacion derecha -->
    <rightJoint>R_joint</rightJoint>
    <!-- distancia entre las ruedas en metros -->
    <wheelSeparation>0.486</wheelSeparation>
    <!-- diametro de las ruedas en metros -->
    <wheelDiameter>0.112</wheelDiameter>
    <!-- acelaracion de las ruedas en rad/s^2-->
    <wheelAcceleration>5.0</wheelAcceleration>
    <!-- torque maximo producido por las ruedas en Nm -->
    <wheelTorque>3</wheelTorque>
    <!-- Topic en el que se recibiran los mensajes geometry_msgs/Twist  -->
    <commandTopic>cmd_vel</commandTopic>
    <!-- Topic en elque se publicaran los mensajes nav_msgs/Odometry que contienen la odometria -->
    <odometryTopic>odom</odometryTopic>
    <!-- frame de referencia para la odometria -->
    <odometryFrame>odom</odometryFrame>
    <!-- frame del robot desde el que se calcula la odometria -->
    <robotBaseFrame>base_link</robotBaseFrame>
    <!-- origen de la odometroa, 0  ENCODER, 1  WORLD -->
    <odometrySource>world</odometrySource>
    <publishTf>1</publishTf>
	<publishOdomTF>true</publishOdomTF>
    <rosDebugLevel>na</rosDebugLevel>
    <!-- true tpara publicar los  "transforms" para las ruedas, defaults  false -->
    <publishWheelTF>false</publishWheelTF>
    <!-- true para publicar "transforms" para la odometria, defaults  true -->
    <publishOdom>true</publishOdom>
    <!-- publicar mensaje sensor_msgs/JointState en el topic /joint_states, defaults  false -->
    <publishWheelJointState>false</publishWheelJointState>
    <!-- Strue tpara ivnertir las ruedas, defaults true -->
    <legacyMode>false</legacyMode>
  </plugin>
</gazebo>
```

Preste atencion a la descripcion de cada elemento en los comentarios del codigo. Si algun cambio se realiza en el robot, debe actualizarse el codigo para que corresponda.

El siguiente plugin a utilizar sera el de agregar un sensor tipo Lidar (radar de luz) que le permite al robot conocer la distacia de cualquier objeto a su alrededor. para esto utilizamos el plugin ***gazebo_ros_head_rplidar_controller*** agregando el siguiente codigo dentro del elemento robot del archivo urdf.
```
<!-- referencia vincula el sensor al link llamado "laser" del modelo urdf -->
<!-- se podria vincular tambien al link "base_link" pero corriendo la posicion -->
<gazebo reference="laser">
    <sensor type="ray" name="head_rplidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <!-- numero de muestras en los 360 grados -->
            <samples>360</samples>
            <!-- resolucion en grados -->
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>
        <range>
          <!-- minima distacia desde la que empieza a medir -->
          <min>0.3</min>
          <!-- maxima distacia que puede medir -->
          <max>8.0</max>
          <!-- resolucion en metros -->
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <!-- conexion al plugin -->
      <plugin name="gazebo_ros_head_rplidar_controller" filename="libgazebo_ros_laser.so">
        <!-- nombre del topic en el que se publicaran los mensajes sensor_msgs/LaserScan -->
        <topicName>scan</topicName>
        <frameName>laser</frameName>
      </plugin>
    </sensor>
</gazebo>
```
preste atencio a los comentarios en cada linea para entender los elementos que parametrizan al plugin.

**actividad: agregar nuevos plugins para cambiar el color de los eslabones**

# Crear transformaciones
Una de los puntos mas confusos de ROS son las transformaciones. Estos son mensajes de tipo tf2_msgs/TFMessage que contienen la posicion de un sistema de referencia con respecto a otro.

Una descripcion detallada de las transformaciones puede encontrarse en este [tutorial](http://wiki.ros.org/tf).

Para nuestro robot es necesario publicar una transformacion (broadcasting a transform) que envie constantemente informacion sobre la posicion del Lidar con respecto a la base del robot. una buena explicacion de porque se requiere esta transformacion se encuentra [aqui](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). Un resumen y adaptacion de esa guia se presenta a continuacion:

Primero navegamos en la terminal hasta la carpeta captbot_ws/src y creamos un nuevo paquete llamado ***capbot_setup_tf***.
```
$ catkin_create_pkg capbot_setup_tf roscpp tf geometry_msgs
```
Ahora vamos a crear un nodo, (hablaremos mas de la creacion de nodos en el siguiente tutorial), para esto entramos a la carpeta ***capbot_setup_tf*** que se acaba de crear y en ella creamos la carpeta ***src***.

```
$ cd capbot_setup_tf
$ mkdir src
```
en la nueva carpeta ***capbot_setup_tf/src*** creamos un nuevo archivo, lo llamamos ***tf_broadcaster.cpp***, abrimos y pegamos el siguiente codigo:

```
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//enviar la informacion de tf del robot
int main(int argc, char** argv){
  ros::init(argc, argv, "capbot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  ROS_INFO("running");
  while(n.ok()){
  //Vector3(0.0, 0.0, 0.3) indica que el lidar esta 30 cm sobre "base_link"
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.3)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
```


Para poder ejecutar este codigo desde ROS, debemos compilar usando catkin_make. Primero abrimos el archivo CMakeLists que esta en la carpeta ***capbot_setup_tf*** y agregamos lo siguiente al final de archivo y guardamos:
```
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
```
Esto le indica a la funcion catkin_make que debe compilar el archivo tf_broadcaster.cpp. 

Ahora ejecutamos:

```
$ cd ~/capbot_ws/
$ catkin_make
```

Al final nos deberia aparecer el mensaje **[100%] Built target tf_broadcaster** indicando que compilo correctamente.


 

