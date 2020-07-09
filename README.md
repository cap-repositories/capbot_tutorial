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
Esto es una formato estandar para guardar el modelo del robot en ros.

Se pueden crear otras carpetas para ir almacenando ordenadamente otros componentes del modelo como archivos de mallas y modelos cad
```
makdir meshes
mkdir materials
mkdir cad
```





Cada aplicaion de ROS debe ejecutarse en su propio workspace para evitar conflictos entre versiones u otros proyectos.
Cuando abrimos el proyecto de ROS en el ROS Development Studio, un workspace de catkin esta preconfigurado y solo debemos verificar que este bien establecido.
En la terminal escribimos:
```
$ ls
```
Aparece la lista de carpetas y archivos, debe aparecer una carpera catkin_ws, entramos a ella con:
```
$ cd catkin_ws
$ ls
```
aparece la lista de carpetas que deben incluir:
* build
* devel
* src
