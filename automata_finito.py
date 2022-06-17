#######################
# Author: Adán Daniel Márquez Hernández
# E-mail: adanmarquez200@outlook.com
# 
# Author: Uzziel Itzak Mesillas Castilla
# E-mail: A01024223
#  Autómata finito 

#  MR2007B.502 Gemelo Digital

# Descripcion

# Automata Finito (AF) || Máquina de Estados Finita (MEF)
# Programación para el proceso de empaquetado del proyecto en la UF MR2007B.502

############################

# Librerías 

from cmath import cos, sin
import os
from shutil import move
import sys
import time
import math
import datetime
import random
import traceback
import threading
import socket
from xarm.wrapper import XArmAPI

# Definición de variables generales 

state = 0 # Variable de estados del AF 
variables = {'move_robot': 0, 'contador': 0}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
camera_data = [0, 0, 0, 0, 0, 0, 0, 0]
'''
    0: x
    1: y
    2: angle
    3: qr
    4: pass/fail color ensamble
    5: pass/fail 4 tornillos
    6: pass/fail color tornillos
    7: tentativo: tornillo tipo allen 
'''
cam_offset_y = 90
cam_offset_x = 10
pick_motor = [0 ,0, 0]
x_center = 512
y_center = 640
x_start = -68.3
y_start = 334.8
conveyor_l_y = 100 # [mm]
conveyor_l_x = 100 # [mm]
conveyor_l_p = 906 # [px]
px_to_mm_y = float(conveyor_l_y/conveyor_l_p)
px_to_mm_x = float(conveyor_l_x/conveyor_l_p)
average_intensity = 0

# Posiciones generales del robot
home_general = [0.0, -24.6, -33.1, 0.0, 60.2, -0.1]
 
# Move arc line: presencia, low, high, pick
ensamble_in_revision = {
    'pos1': [False, [274.8, -172.1, 223, 180, 0, -44.6], [274.8, -172.1, 345, 180, 0, -44.6], [274.8, -172.1, 215, 180, 0, -44.6]],
    'pos2': [False, [325.3, -96.3, 223, 180, 0, -49.7], [325.3, -96.3, 345, 180, 0, -49.7], [325.3, -96.3, 215, 180, 0, -49.7]],
    'pos3': [False, [222.8, -96.8, 223, 180, 0, -46], [222.8, -96.8, 345, 180, 0, -46], [222.8, -96.8, 215, 180, 0, -46]],
    'pos1cam': [184.3, -165.1, 345, 180, 0, 0],
    'pos2cam': [239.9, -86.1, 345, 180, 0, 0],
    'pos3cam': [132.2, -88.1, 345, 180, 0, 0],
    
}

ensamble_in_paletizado1 = {
    'pos1': [False, [308.2, 41, 220.9, 180, 0, -5.2], [308.2, 41, 345, 180, 0, -5.2]],
    'pos2': [False, [252.2, 41, 220.9, 180, 0, -5.2], [252.2, 41, 345, 180, 0, -5.2]],
    'pos3': [False, [308.2, -15, 220.9, 180, 0, -5.2], [308.2, -15, 345, 180, 0, -5.2]],
    'pos4': [False, [252.2, -15, 220.9, 180, 0, -5.2], [252.2, -15, 345, 180, 0, -5.2]],
    }

ensamble_in_paletizado2 = {
    'pos1': [False, [371.6, 165.2, 220.9, 180, 0, -2.8], [371.6, 165.2, 345, 180, 0, -2.8]],
    'pos2': [False, [315.6, 165.2, 220.9, 180, 0, -2.8], [315.6, 165.2, 345, 180, 0, -2.8]],
    'pos3': [False, [371.6, 109.2, 220.9, 180, 0, -2.8], [371.6, 109.2, 345, 180, 0, -2.8]],
    'pos4': [False, [315.6, 109.2, 220.9, 180, 0, -2.8], [315.6, 109.2, 220.9, 180, 0, -2.8]],
    }

ensamble_in_paletizado3 = {
    'pos1': [False, [250.3, 164.1, 220.9, 180, 0, -1.8], [250.3, 164.1, 345, 180, 0, -1.8]],
    'pos2': [False, [194.3, 164.1, 220.9, 180, 0, -1.8], [194.3, 164.1, 345, 180, 0, -1.8]],
    'pos3': [False, [250.3, 108.1, 220.9, 180, 0, -1.8], [250.3, 108.1, 345, 180, 0, -1.8]],
    'pos4': [False, [194.3, 108.1, 220.9, 180, 0, -1.8], [194.3, 108.1, 345, 180, 0, -1.8]],
    }

# Datos del servidor de la cámara
HOST = "192.168.1.126"  # IP Cámara estación 6
#HOST = "127.0.0.5" # Emulador de de cámara estación 6
PORT = 20000  # The port used by the server


def hangle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))
    # TODO：Do different processing according to the error code

# variables del robot

arm = XArmAPI('192.168.1.206', do_not_open=True)
arm.register_error_warn_changed_callback(hangle_err_warn_changed)
arm.connect()

# enable motion
arm.motion_enable(enable=True)
# set mode: position control mode
arm.set_mode(0)
# set state: sport state
arm.set_state(state=0)

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

def move_axis(lista):
     if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*lista, speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))

def move_angle(lista):
    if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=lista, speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))

def get_data_from_camera():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(20000)
        try:
            s.connect((HOST, PORT))
            data = s.recv(150)
        except:
            print("No se recibió data de la cámara")
            data = ["0; 0; 0; 0; 0; 0; 0; 0"]

    # String manipulation
    data_str = str(data)
    data_str = data_str.split(";")

    print("TCP output ", data_str)

    x_motor = float(data_str[1])
    y_motor = data_str[0]
    try:
        y_motor = float(y_motor[-3:])
    except:
        y_motor = 0

    angle_ref = float(data_str[2])
    # Cálculo a mm con respecto al centro de cámara
    x_motor -= x_center
    y_motor -= y_center

    # #Cálculo de ángulo (solo primer cuadrante)
    if angle_ref >= 0 and angle_ref <= 90:
        angle = 90 - angle_ref
    elif angle_ref < 0 and angle_ref >= -90:
        angle = abs(angle_ref) + 90

    x_mm = x_motor*px_to_mm_x
    y_mm = y_motor*px_to_mm_y

    print('Posición en x: ', x_mm)
    print('Posición en y: ', y_mm)
    print('Ángulo: ', angle)

    qr = float(data_str[3])
    four_tornillos = float(data_str[4])
    color_tornillos = float(data_str[5])
    color_motor = float(data_str[6])
    color_reductor = data_str[7]
    try:
        color_reductor = float(color_reductor[:6])
    except:
        color_reductor = 0
    if data == [0, 0, 0, 0, 0, 0, 0, 0]:
        print('No hubo lectura de la cámara')
    else:
        print("Lectura exitosa de datos ")

    return x_mm, y_mm, angle, qr, four_tornillos, color_tornillos, color_motor, color_reductor
    
def take_photo():
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_cgpio_digital(5, 1, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_cgpio_digital, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        arm.set_pause_time(1)
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_cgpio_digital(5, 0, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_cgpio_digital, code={}'.format(code))

def close_gripper():
    if arm.error_code == 0 and not params['quit']:
        arm.set_pause_time(0.5)
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_cgpio_digital(0, 1, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_cgpio_digital, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        arm.set_pause_time(0.5)
    if arm.error_code == 0 and not params['quit']:
        arm.set_pause_time(1)

def open_gripper():
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_cgpio_digital(0, 0, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_cgpio_digital, code={}'.format(code))
    if arm.error_code == 0 and not params['quit']:
        arm.set_pause_time(1)

def pick_up_motor():
    global camera_data
    camera_data = get_data_from_camera()
    # Asignando offset
    print("Camera data inside pick_up_motor: ", camera_data)
    pick_motor[0] = x_start + cam_offset_x + camera_data[0] 
    pick_motor[1] = y_start + cam_offset_y + camera_data[1] 
    pick_motor[2] = camera_data[2]
    move_axis([-68.3, pick_motor[1], 434.4, -180, 0, 90])
    move_axis([pick_motor[0], pick_motor[1], 434.4, -180, 0, 90])
    move_axis([pick_motor[0], pick_motor[1], 434.4, -180, 0, pick_motor[2]])
    move_axis([pick_motor[0], pick_motor[1], 250, -180, 0, pick_motor[2]])
    move_axis([pick_motor[0], pick_motor[1], 230, -180, 0, pick_motor[2]])
    close_gripper()
    move_axis([pick_motor[0], pick_motor[1], 300, -179.7, 1.2, pick_motor[2]])
   

# Tomar ensamble correcto
def tom_co_pos(color_motor, color_reductor):
    #global camera_data
    print('State: tom_co_pos')
    print('Moviendo ensamble')
    if (color_motor > 12 and color_motor < 40) and (color_reductor > 12 and color_reductor < 40):
        print('Ensamble rojo detectado')
        if ensamble_in_revision['pos1'][0] == False:
            print('Moviendo a base roja...')

            pick_up_motor()

            move_angle(home_general)
            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_revision['pos1'][1])

            open_gripper()

            move_axis(ensamble_in_revision['pos1'][2])
            move_angle(home_general)

            ensamble_in_revision['pos1'][0] = True
            if ensamble_in_revision['pos1'][0] == True and ensamble_in_revision['pos2'][0] == True and ensamble_in_revision['pos3'][0] == True:
                revision_ensamble()
            else:
                recepcion_ensambles()
        else:
            print('Ya existe un ensamble en la base roja')
            print('Proceder a revisión de ensambles en las bases')
            move_angle(home_general)

            revision_ensamble()
    elif color_motor >= 40 and color_reductor >= 40:
        print('Ensamble amarillo detectado')
        if ensamble_in_revision['pos2'][0] == False:
            print('Moviendo a base amarilla...')

            pick_up_motor()

            move_angle(home_general)
            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_revision['pos2'][1])

            open_gripper()

            move_axis(ensamble_in_revision['pos2'][2])
            move_angle(home_general)

            ensamble_in_revision['pos2'][0] = True
            if ensamble_in_revision['pos1'][0] == True and ensamble_in_revision['pos2'][0] == True and ensamble_in_revision['pos3'][0] == True:
                revision_ensamble()
            else:
                recepcion_ensambles()
        else:
            print('Ya existe un ensamble en la base amarilla')
            print('Proceder a revisión de ensambles en las bases')
            move_angle(home_general)
            revision_ensamble()
    elif color_motor < 12 and color_reductor < 12:
        print('Ensamble verde detectado')
        if ensamble_in_revision['pos3'][0] == False:
            print('Moviendo a base verde...')

            pick_up_motor()

            move_angle(home_general)
            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_revision['pos3'][1])

            open_gripper()

            move_axis(ensamble_in_revision['pos3'][2])
            move_angle(home_general)

            ensamble_in_revision['pos3'][0] = True
            if ensamble_in_revision['pos1'][0] == True and ensamble_in_revision['pos2'][0] == True and ensamble_in_revision['pos3'][0] == True:
                revision_ensamble()
            else:
                recepcion_ensambles()
        else:
            print('Ya existe un ensamble en la base verde')
            print('Proceder a revisión de ensambles en las bases')
            move_angle(home_general)
            revision_ensamble()
    else:
        print('El color del ensamble no es el mismo')

        pick_up_motor()

        move_angle([100.1, -29.9, -31.1, 0, 61, 12.8])
        move_angle([-80, -29.9, -31.1, 0, 61, 12.8])
        open_gripper()
        move_angle(home_general)

        recepcion_ensambles()

def take_out_ensamble():
    print('State: take_out_ensamble')
    print('Colocando ensamble fuera de la línea de producción...')

    pick_up_motor()
    # agregar coordenadas para poner fuera de la línea de producción

    move_angle([100.1, -29.9, -31.1, 0, 61, 12.8])
    move_angle([-80, -29.9, -31.1, 0, 61, 12.8])
    open_gripper()
    move_angle(home_general)
    recepcion_ensambles()

def get_coordinates():
    print('State: get_coordinates')
    move_axis([-68.3, 334.8, 434.4, -180, 0, 90])
    global camera_data
    print("Cam data inside get_coordinates: ", camera_data)
    qr_value = camera_data[3]
    motor_intensity = camera_data[6]
    reductor_intensity = camera_data[7]
    
    take_photo()
    print('Tomando fotos...')
    

    if float(qr_value) == 1:
        tom_co_pos(motor_intensity, reductor_intensity)
    else:
        take_out_ensamble()

def move_to_paletizado():
    print('State: move_to_paletizado')
    '''
    Estado en el cual, luego de la revisión, se procede a llevar un ensamble correcto a la posición
    más libre de su respectivo color.
    Luego de juntar 4 ensambles del mismo color, se procede a notificar al operario para que se lleve
    y empaque dichos ensambles
    '''
    global ensamble_in_revision, ensamble_in_paletizado1
    global ensamble_in_paletizado2, ensamble_in_paletizado3
    if ensamble_in_revision['pos1'][0] == True:
        print('Moviendo ensamble rojo a zona de Paletizado')
        # Mover ensamble blanco al grupo de 4
        if not ensamble_in_paletizado1['pos1'][0]:
            # Mover a la posición 1
            print('Ensamble rojo a pos 1')
            ensamble_in_paletizado1['pos1'][0] = True

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_revision['pos1'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_paletizado1['pos1'][2])
            move_axis(ensamble_in_paletizado1['pos1'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado1['pos1'][2])
            move_angle(home_general)
            ensamble_in_revision['pos1'][0] = False
            revision_ensamble()

        elif not ensamble_in_paletizado1['pos2'][0]:
            print('Ensamble rojo a pos 2')
            print('Moviendose a: ', ensamble_in_paletizado1['pos2'][1])
            ensamble_in_paletizado1['pos2'][0] = True

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_revision['pos1'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_paletizado1['pos2'][2])
            move_axis(ensamble_in_paletizado1['pos2'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado1['pos2'][2])
            move_angle(home_general)
            ensamble_in_revision['pos1'][0] = False
            revision_ensamble()
        elif not ensamble_in_paletizado1['pos3'][0]:
            print('Ensamble rojo a pos 3')
            print('Moviendose a: ', ensamble_in_paletizado1['pos3'][1])
            ensamble_in_paletizado1['pos3'][0] = True

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_revision['pos1'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_paletizado1['pos3'][2])
            move_axis(ensamble_in_paletizado1['pos3'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado1['pos3'][2])
            move_angle(home_general)
            ensamble_in_revision['pos1'][0] = False
            revision_ensamble()
        elif not ensamble_in_paletizado1['pos4'][0]:
            print('Ensamble rojo a pos 4')
            print('Moviendose a: ', ensamble_in_paletizado1['pos4'][1])
            ensamble_in_paletizado1['pos4'][0] = True

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_revision['pos1'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos1'][2])
            move_axis(ensamble_in_paletizado1['pos4'][2])
            move_axis(ensamble_in_paletizado1['pos4'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado1['pos4'][2])
            move_angle(home_general)
            ensamble_in_revision['pos1'][0] = False
            # Mandar mensaje de paletizado completo 
            print('Proceder a embalaje de Ensamble rojo')
            move_angle(home_general)
            ensamble_in_paletizado1['pos1'][0] = False
            ensamble_in_paletizado1['pos2'][0] = False
            ensamble_in_paletizado1['pos3'][0] = False
            ensamble_in_paletizado1['pos4'][0] = False
            revision_ensamble()
        ensamble_in_revision['pos1'][0] = False
    if ensamble_in_revision['pos2'][0] == True:
        print('Moviendo ensamble amarillo a zona de Paletizado')
        # Mover ensamble azul al grupo de 4
        if not ensamble_in_paletizado2['pos1'][0]:
            # Mover a la posición 1
            print('Ensamble amarillo a pos 1')
            print('Moviendose a: ', ensamble_in_paletizado2['pos1'][1])
            ensamble_in_paletizado2['pos1'][0] = True

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_revision['pos2'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_paletizado2['pos1'][2])
            move_axis(ensamble_in_paletizado2['pos1'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado2['pos1'][2])
            move_angle(home_general)
            ensamble_in_revision['pos2'][0] = False
            revision_ensamble()
        elif not ensamble_in_paletizado2['pos2'][0]:
            print('Ensamble amarillo a pos 2')
            print('Moviendose a: ', ensamble_in_paletizado2['pos2'][1])
            ensamble_in_paletizado2['pos2'][0] = True

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_revision['pos2'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_paletizado2['pos2'][2])
            move_axis(ensamble_in_paletizado2['pos2'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado2['pos2'][2])
            move_angle(home_general)
            ensamble_in_revision['pos2'][0] = False
        elif not ensamble_in_paletizado1['pos3'][0]:
            print('Ensamble amarillo a pos 3')
            print('Moviendose a: ', ensamble_in_paletizado2['pos3'][1])
            ensamble_in_paletizado2['pos3'][0] = True

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_revision['pos2'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_paletizado2['pos3'][2])
            move_axis(ensamble_in_paletizado2['pos3'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado2['pos3'][2])
            move_angle(home_general)
            ensamble_in_revision['pos2'][0] = False
        elif not ensamble_in_paletizado2['pos4'][0]:
            print('Ensamble amarillo a pos 4')
            print('Moviendose a: ', ensamble_in_paletizado2['pos4'][1])
            ensamble_in_paletizado2['pos4'][0] = True

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_revision['pos2'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos2'][2])
            move_axis(ensamble_in_paletizado2['pos4'][2])
            move_axis(ensamble_in_paletizado2['pos4'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado2['pos4'][2])
            move_angle(home_general)
            ensamble_in_revision['pos2'][0] = False
            # Mandar mensaje de paletizado completo 
            print('Proceder a embalaje de Ensamble amarillo')
            ensamble_in_paletizado2['pos1'][0] = False
            ensamble_in_paletizado2['pos2'][0] = False
            ensamble_in_paletizado2['pos3'][0] = False
            ensamble_in_paletizado2['pos4'][0] = False
            revision_ensamble()
        ensamble_in_revision['pos2'][0] = False
    if ensamble_in_revision['pos3'][0] == True:
        print('Moviendo ensamble verde a zona de Paletizado')
        # Mover ensamble morado al grupo de 4
        if not ensamble_in_paletizado3['pos1'][0]:
            print('Ensamble verde a pos 1')
            print('Moviendose a: ', ensamble_in_paletizado3['pos1'][1])
            ensamble_in_paletizado3['pos1'][0] = True

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_revision['pos3'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_paletizado3['pos1'][2])
            move_axis(ensamble_in_paletizado3['pos1'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado3['pos1'][2])
            move_angle(home_general)
            ensamble_in_revision['pos3'][0] = False
        elif not ensamble_in_paletizado3['pos2'][0]:
            print('Ensamble verde a pos 2')
            print('Moviendose a: ', ensamble_in_paletizado3['pos2'][1])
            ensamble_in_paletizado3['pos2'][0] = True

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_revision['pos3'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_paletizado3['pos2'][2])
            move_axis(ensamble_in_paletizado3['pos2'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado3['pos2'][2])
            move_angle(home_general)
            ensamble_in_revision['pos3'][0] = False
        elif not ensamble_in_paletizado3['pos3'][0]:
            print('Ensamble verde a pos 3')
            print('Moviendose a: ', ensamble_in_paletizado3['pos3'][1])
            ensamble_in_paletizado3['pos3'][0] = True

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_revision['pos3'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_paletizado3['pos3'][2])
            move_axis(ensamble_in_paletizado3['pos3'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado3['pos3'][2])
            move_angle(home_general)
            ensamble_in_revision['pos3'][0] = False
        elif not ensamble_in_paletizado3['pos4'][0]:
            print('Ensamble verde a pos 4')
            print('Moviendose a: ', ensamble_in_paletizado3['pos4'][1])
            ensamble_in_paletizado1['pos1'][0] = True

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_revision['pos3'][3])

            close_gripper()

            move_axis(ensamble_in_revision['pos3'][2])
            move_axis(ensamble_in_paletizado3['pos4'][2])
            move_axis(ensamble_in_paletizado3['pos4'][1])

            open_gripper()

            move_axis(ensamble_in_paletizado3['pos4'][2])
            move_angle(home_general)
            ensamble_in_revision['pos3'][0] = False
            # Mandar mensaje de paletizado completo 
            print('Proceder a embalaje de Ensamble verde')
            ensamble_in_paletizado3['pos1'][0] = False
            ensamble_in_paletizado3['pos2'][0] = False
            ensamble_in_paletizado3['pos3'][0] = False
            ensamble_in_paletizado3['pos4'][0] = False
            revision_ensamble()
        ensamble_in_revision['pos3'][0] = False
    print('to recepción de motores')
    recepcion_ensambles()

def revision_ensamble():
    '''
    Estado en el cual el robot se posiciona para tomar fotos y analizar a los ensambles.
    Se tomará una foto superior para analizar:
        Tornillos completos (4)
        Tornillos tipo Allen
        Tornillos del color asignado al modelo
    Se tomará otra foto lateral para analizar:
        Mismo color de motor y reductor
        Mismo modelo de motor y reductor
    Luego de verificar que ensambles cumplen se procede a colocar los correcto en la zona de paletizado
    y los demás se colocan fuera del área de trabajo
    '''
    print('State: Revisión')

    global camera_data
    global ensamble_in_revision
    while True:
        if ensamble_in_revision['pos1'][0] == True:
            move_axis(ensamble_in_revision['pos1cam'])
            print('Tomando foto...')
            take_photo()
            camera_data = get_data_from_camera()
            screw_all = camera_data[4]
            screw_color = camera_data[5]

            if screw_all == 1 and screw_color == 1:
                move_to_paletizado()
            else:
                print('Ensamble no cumple condiciones')
                move_axis(ensamble_in_revision['pos1'][2])
                move_axis(ensamble_in_revision['pos1'][1])
                close_gripper()
                move_axis(ensamble_in_revision['pos1'][2])
                move_axis([55, -319, 345, 180, 0, -92.8])
                open_gripper()
                move_angle(home_general)
                ensamble_in_revision['pos1'][0] = False
        elif ensamble_in_revision['pos2'][0] == True:
            move_axis(ensamble_in_revision['pos2cam'])
            print('Tomando foto...')
            take_photo()
            camera_data = get_data_from_camera()
            screw_all = camera_data[4]
            screw_color = camera_data[5]

            if screw_all == 1 and screw_color == 1:
                move_to_paletizado()
            else:
                print('Ensamble no cumple condiciones')
                move_axis(ensamble_in_revision['pos2'][2])
                move_axis(ensamble_in_revision['pos2'][1])
                close_gripper()
                move_axis(ensamble_in_revision['pos2'][2])
                move_axis([55, -319, 345, 180, 0, -92.8])
                open_gripper()
                move_angle(home_general)
                ensamble_in_revision['pos2'][0] = False
        elif ensamble_in_revision['pos3'][0] == True:
            move_axis(ensamble_in_revision['pos3cam'])
            print('Tomando foto...')
            take_photo()
            camera_data = get_data_from_camera()
            screw_all = camera_data[4]
            screw_color = camera_data[5]

            if screw_all == 1 and screw_color == 1:
                move_to_paletizado()
            else:
                print('Ensamble no cumple condiciones')
                move_axis(ensamble_in_revision['pos3'][2])
                move_axis(ensamble_in_revision['pos3'][1])
                close_gripper()
                move_axis(ensamble_in_revision['pos3'][2])
                move_axis([55, -319, 345, 180, 0, -92.8])
                open_gripper()
                move_angle(home_general)
                ensamble_in_revision['pos3'][0] = False
        else:
            recepcion_ensambles()

def recepcion_ensambles():
    '''
    Estado en el cual se reciben ensambles de la banda transportadora y son colocados en su base
    correspondiente dependiendo del color de estos ensambles
    Se espera que el color sea detectado por medio de un código de barras, que indique
    tanto si es motor o reductor, su modelo, y su color
    '''
    print('State: Recepcion')
    print('Esperando ensamble en la banda transportadora...')

    # Parámetros iniciales del Robot

    if not params['quit']:
        params['speed'] = 180
    if not params['quit']:
        params['acc'] = 200
    if arm.error_code == 0 and not params['quit']:
        code = arm.set_cgpio_digital(0, 0, delay_sec=0)
        if code != 0:
            params['quit'] = True
            pprint('set_cgpio_digital, code={}'.format(code))
    if not params['quit']:
        params['variables']['contador'] = 0
    if not params['quit']:
        params['variables']['move_robot'] = False
    if not params['quit']:
        arm.set_tcp_offset([6.3, 0, 0, 0, 0, 0], wait=True)
        arm.set_state(0)
        time.sleep(0.5)

    # Loop de recepción de ensambles

    while True:
        if params['quit']:
            break
        if arm.get_cgpio_digital(0)[1] == 0 and params['variables'].get('move_robot', 0) == True:
            if arm.error_code == 0 and not params['quit']:
                arm.set_pause_time(1)
            # home general del robot
            # move_angle(home_general)

            # Tomar fotos para detectar QR del ensamble 

            move_axis([240.2, 331.2, 88, -90, 0, 90])
            take_photo()
            print('Tomando foto...')
            global camera_data
            camera_data = get_data_from_camera()
            camera_qr = camera_data[3]
            if camera_qr == 1:
                print('El ensamble pertence a la línea de producción')
                get_coordinates()
            else:
                print('QR no detectado')
                move_axis([254.2, 454.1, 88, -90, 0, 111])
                take_photo()
                print('Tomando foto...')
            
                camera_data_2 = get_data_from_camera()
                camera_qr = camera_data_2[3]
                if camera_qr == 1:
                    print('El ensamble pertence a la línea de producción')
                    get_coordinates()
                else:
                    print('El ensamble no pertenece a la línea de producción')
                    get_coordinates()
        elif arm.get_cgpio_digital(0)[1] == 1:
            if not params['quit']:
                params['variables']['move_robot'] = True
                pick_motor[0] = 0
                pick_motor[1] = 0
                pick_motor[2] = 0            

def main():
    print('Inicializando sección de Empaque y Embarque...')
    #print('Inicializando variables iniciales...')

    sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
    #######################################################

    # Test connection to the robot

    if len(sys.argv) >= 2:
        ip = sys.argv[1]
    else:
        try:
            from configparser import ConfigParser
            parser = ConfigParser()
            parser.read('../robot.conf')
            ip = parser.get('xArm', 'ip')
        except:
            #ip = input('Please input the xArm ip address:')
            ip = "192.168.1.206"
            print('xArm IP address : ', ip)
            if not ip:
                print('input error, exit')
                sys.exit(1)
    print('\n')

    recepcion_ensambles()

main()