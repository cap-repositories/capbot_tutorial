para instalar ROS se recomienda usar una maquina virtual en windows. Siempre es mejor tener una particion de linux pero resulta mas facil y rapido usar la maquina virtual.

El procedimiento es el siguiente:
1. Descargar e instalar VMware Workstation 16 Player [aqui](https://www.vmware.com/co/products/workstation-player/workstation-player-evaluation.html). el software es gratuito para fines no comerciales
2. Descargar Ubuntu Mate 20.04.1 [aqui](https://ubuntu-mate.org/download/amd64/focal/).
3. intalar Ubuntu Mate en el VMware.
4. En la nueva instalacion de ubuntu, se debe instalar ROS abriendo una terminal y siguiendo los pasos de la guia de instalacion de ROS [aqui](http://wiki.ros.org/Installation/Ubuntu). A continuacion se resume el proceso de instalacion:

Abra una nueva terminal y copie los siguientes comandos en orden.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Los siguientes comando puede tardes bastante
```
sudo apt update
```

```
sudo apt install ros-noetic-desktop-full
```
Finalmente, se debe correr el archivo setup.bash de ROS cada que se abre una terminal. Para que esto se haga de forma automatica cada que se abra una nueva terminal, se deben correr los siguientes comandos en la terminal:
* ``` echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc ```
* ``` source ~/.bashrc ```

Pruebe la instalacion iniciando al ambiente de simualacion **Gazebo**, en la terminar escriba ***gazebo*** y presiones enter. Si la aplicacion abre y se mantiene, todo quedo bien. Si al aplicacion se cierra y muestra un error en consola, ejecute el siguiente comando:
```
echo "export SVGA_VGPU10=0" >> ~/.profile
```

