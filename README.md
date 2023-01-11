# Teleoperación Hector Quadrotor simulado en Gazebo 

## Índice
1. [Índice](#índice)
2. [Conexión Arduino](#conexión-arduino)
3. [Requerimientos previos](#requerimientos-previos)
4. [Lanzamiento de la simulación](#lanzamiento-de-la-simulación )
5. [Lanzamiento de la teleoperación](#lanzamiento-de-la-teleoperación)

## Conexión Arduino

![MPU-6050 GY-521](https://github.com/paablomrtinez/drone_arduino_teleop/blob/main/Assets/IMU.jpeg)

![Cableado para la conexión entre la IMU y la placa Arduino UNO](https://github.com/paablomrtinez/drone_arduino_teleop/blob/main/Assets/conexion_IMU_Arduino.jpeg)

![Joystick](https://github.com/paablomrtinez/drone_arduino_teleop/blob/main/Assets/joystick.jpeg)

## Requerimientos previos 

Creación de un espacio de trabajo donde se alberguen los paquetes correspondientes al dron y a la teleoperación

  $ mkdir -p catkin_drone/src
  
Dentro del workspace creado, seguir el siguiente tutorial con el fin de instalar los requerimientos para la simulación del dron:

  https://github.com/RAFALAMAO/hector-quadrotor-noetic
  
Construir el espacio de trabajo antes de lanzar los nuevos paquetes instalados:
	
	$ catkin build 
  
IMPORTANTE: Siempre que se abra un terminal y se trabaje en 'catkin_drone':

	$ source devel/setup.bash
  
## Lanzamiento de la simulación 

Simulación básica con dron en un mundo vacío:

	$ roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
	
Para lanzar la simulación de otros entornos se pueden utilizar las demos que contiene el paquete del hector quadrotor:

	$ roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch

## Lanzamiento de la teleoperación 

Para la teleoperación con la IMU y el Joystick se ha de abrir previamente la comunicación serie por el puerto en el que se encuentre la placa con el programa ejecutándose en ella. El monitor Serie de este debe mantenerse cerrado: 

	$ rosrun rosserial_python serial_node.py /dev/ttyACM0

Para lanzar el nodo de teleoperación con la IMU, dentro de 'catkin_drone':

	$ rosrun imu_teleop_drone IMU_teleop_drone.py 
  
