# capbot_tutorial
___


## Configurar el Workspace 

```
$ mkdir capbot_ws/src
$ cd capbot_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
```
##Preparar el Modelo del robot

Crear paquete con dependencia de urdf (para visualizar el modelo desde ros)
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg capbot_description urdf
$ cd capbot_description
```
Crear una carpeta /urdf para almacenar el archivo urdf del modelo del robot

```
$ mkdir urdf
```
Esto es una formato estandar para guardar el modelo del robot en ros. en esrta carpeta se debe poner el archivo capbot.urdf

Se pueden crear otras carpetas para ir almacenando ordenadamente otros componentes del modelo como archivos de mallas y modelos cad
```
makdir meshes
mkdir materials
mkdir cad
```
Cree una carpeta **src/** y en un editor de texto cree un archivo llamado **parser.cpp** y guardelo en la nueva carpeta **src/**

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
Para ejecutar el codigo, primero agregue las siguientes líneas al archivo CMakeList.txt:
```
 add_executable(parser src/parser.cpp)
 target_link_libraries(parser ${catkin_LIBRARIES})
 ```

construya el paquete y ejecútelo.
```
$ catkin_make
$ ./devel/lib/capbot_description/parser ./src/capbot_description/urdf/capbot.urdf
```
Si el analisis fue correcto se presentara un mensaje "Successfully parsed urdf file", de lo contrario se mostrara el error detectado.


