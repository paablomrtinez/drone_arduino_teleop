#!/usr/bin/env python3

from __future__ import print_function

import threading

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Topics a través de los que se recibe o publica información
imu_topic = '/imu'
velocity_topic = '/cmd_vel'                         # simulación

# Inicialización de variables
alpha = 0
beta = 0
joystick = 0
mode = 0            # Cambia entre modo normal (0) u holonómico (1)
prev_but = 1

# Velocidades por defecto
v_low = 0.3 
v_high = 0.8

w_low = 0.2
w_high = 0.4

# Ganancia proporcional la inclinación para calcular velocidades
step_size_speed = 0.035
step_size_turn = 0.09
step_size_elevation = 0.005

# Cambios mínimos para asignar nuevas velocdidades
min_change = 0.2
joy_change = 0.2

# Offsets de los giros 
offset_avance = 9.2
offset_giro = 1.5
offset_joystick = 500      # En la posición central, va de [0,1000]

# Inicialización de velocidades de control
control_speed = 0
control_turn = 0
movement = {"x": 0, "y":0, "z": 0, "th": 0}

pub = rospy.Publisher(velocity_topic, Twist, queue_size=10)

msg = """
Reading from the IMU and Publishing to Twist!
---------------------------
Moving around: (IMU axes) 

         ______ Y        forward
        |                   |   
        |                   |___ turn
        |
          X

For Holonomic mode (strafing), hold down the button:
---------------------------

         ______ Y        forward
        |                   |   
        |                   |___ turn
        |
          X

Joystick : up (+z) / down (-z)

Velocity depends proporcionally on IMU & joystick data

In order to change mode, press de Joystick

CTRL-C to quit
"""

def publish_vel(alpha, beta, joystick):
    """	
	Function which send the command velocity to robot according to the changes in IMU
	PITCH AND YAW values
	Args:
		alpha:      Roll
		beta:       Pitch
        joystick:   X axis 
	Return:
		The function is not returning anything
	"""
    try:
        #Declaring as global to access these values in each loop
        global pub
        global control_speed
        global control_turn 
        global movement

        #r = rospy.Rate(2.5)		# Se define la frecuencia de ros para no saturar al robot

        # Converting into integer
        alpha = int(alpha)
        beta = int(-beta)
        joystick = int(joystick)

        control_speed = round(step_size_speed * beta,2)
        control_turn = round(step_size_turn * alpha,2)
        control_elevation = round(step_size_elevation * joystick,2)

        twist = Twist()

        # Asignación de velocidades para publicar: 
        # De avance (en X)
        if abs(movement["x"] - control_speed) > min_change: 
            
            movement["x"] = control_speed
        
        # De elvación (en Z)
        if abs(movement["z"] - control_elevation) > joy_change: 
            
            movement["z"] = control_elevation
        
        # En función del modo los giros son con velocidad en Y (holonómico)
        # o modificando la angular en Z
        if mode == 0: 

            movement["y"] = 0
            if abs(movement["th"] - control_turn) > min_change: 

                movement["th"] = control_turn

        elif mode == 1: 

            movement["th"] = 0
            if abs(movement["y"] - control_turn) > min_change: 
                 
                movement["y"] = control_turn

        else: 
            rospy.loginfo("Incorrect MODE")

        # Si se tienen valores de velocidad muy pequeños se redondean a 0
        if abs(movement["x"]) < 0.2: movement["x"] = 0
        if abs(movement["y"]) < 0.2: movement["y"] = 0
        if abs(movement["z"]) < 0.2: movement["z"] = 0
        if abs(movement["th"]) < 0.2: movement["th"] = 0

        rospy.loginfo("Speed ->  X: {}; Y: {}; Z: {}; Rot: {}".format(movement["x"], movement["y"], movement["z"], movement["th"],) )

        # Asigación final de velocidades para publicar por el topic
        twist.linear.x = movement["x"] 
        twist.linear.y = movement["y"] 
        twist.linear.z = movement["z"] 
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = movement["th"] 

        # Publish.
        pub.publish(twist)

    except TypeError:

        print("No se han recibido datos correctos de la IMU, \
               probablemente se haya perdido la conexión")

def imu_callback(imu):

    """
    Recibe datos del topic en el que publica arduino, en él se encuentra
        - Orientaciones de la IMU
        - Valor de un Joystick
        - Valor del botón que incorpora el joystick

    params: 
        :arg data: String con orientaciones en x e y, valor del joystic y el botón
                   -> 'data: "AxBvCjoyDbutE"'
    """
    #rospy.loginfo(imu.data)

    Ax = float(imu.data[imu.data.index('A')+1:imu.data.index('B')])
    Ay = float(imu.data[imu.data.index('B')+1:imu.data.index('C')])
    Joy_value = float(imu.data[imu.data.index('C')+1:imu.data.index('D')])
    But = float(imu.data[imu.data.index('D')+1:imu.data.index('E')])

    global alpha, beta, mode, joystick, prev_but

    # Si se ha pulsado el botón se cambia de modo
    if prev_but != But and But == 0: 
        
        mode = not mode
        prev_but = But

        rospy.loginfo("Mode changed")

    elif But == 1: 
        prev_but = But
    
	# Se calculan los valores restando un offset 
    alpha = Ax - offset_avance
    beta = Ay - offset_giro
    joystick = Joy_value - offset_joystick

	# Función para publicar las velocidades
    publish_vel(alpha, beta, joystick)

def move_by_imu():

    # Se configura la parte de Subscriber 
    rospy.init_node("imu_teleop_drone", anonymous=True)
    rospy.Subscriber(imu_topic, String, imu_callback) # myMessage

    # Nodo 'infinito'
    rospy.spin()

if __name__=="__main__":

    move_by_imu()
