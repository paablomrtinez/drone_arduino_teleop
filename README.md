# Teleoperación Hector Quadrorotor simulado en Gazebo 

Se ha llevado a cabo la teleoperación de un dron simulado en Gazebo a partir de los datos de una IMU y un Joystick procesados con Arduino. 

## Conexión Arduino

## Requerimientos previos 

Creación de un espacio de trabajo donde se alberguen los paquetes correspondientes al dron y a la teleoperación

  $ mkdir -p catkin_drone/src
  
Dentro del workspace creado, seguir el siguiente tutorial con el fin de instalar los requerimientos para la simulación del dron:

  https://github.com/RAFALAMAO/hector-quadrotor-noetic
  
Construir el espacio de trabajo antes de lanzar los nuevos paquetes instalados:
	
	$ catkin build 
  
IMPORTANTE: Siempre que se abra un terminal y se trabaje en 'catkin_drone':

	$ source devel/setup.bash
  
## Lanzamiento de la simulación del dron

Simulación básica con dron en un mundo vacío:

	$ roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
	
Para lanzar la simulación de otros entornos se pueden utilizar las demos que contiene el paquete del hector quadrorotor:

	$ roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

## Lanzamiento del nodo de teleoperación 

Para la teleoperación con la IMU y el Joystick se ha de abrir previamente la comunicación serie por el puerto en el que se encuentre la placa con el programa ejecutándose en ella. El monitor Serie de este debe mantenerse cerrado: 

	$ rosrun rosserial_python serial_node.py /dev/ttyACM0

Para lanzar el nodo de teleoperación con la IMU, dentro de 'catkin_drone':

	$ rosrun imu_teleop_drone IMU_teleop_drone.py 
  
