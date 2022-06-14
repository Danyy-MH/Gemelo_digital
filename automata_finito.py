#######################
# Adán Márquez
# A00827172@tec.mx
#  Autómata finito 

#  MR2007B.502 Gemelo Digital

# Descripcion

# Automata Finito (AF) || Máquina de Estados Finita (MEF)
# Programación para el proceso de empaquetado del proyecto en la UF MR2007B.502

############################

# Librerías 

from cmath import cos, sin
import os
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
camera_data = [0, 0, 0, 0, 0]
sensor_x = float(-110.0)
sensor_y = float(369)
pick_motor = [0 ,0, 0]
comp_x = 0
comp_y = 0
 
ensamble_in_revision = {
    'pos1': [False, [272.8 , -170.9, 213.6, 180, 0, -43.5]],
    'pos2': [False, [1, 2, 3, 4, 5, 6]],
    'pos3': [False, [1, 2, 3, 4, 5, 6]],
}

ensamble_in_paletizado1 = {
    'pos1': [False, [1, 2, 3, 4, 5, 6]],
    'pos2': [False, [1, 1, 1, 1, 1, 1]],
    'pos3': [False, [2, 2, 2, 2, 2, 2]],
    'pos4': [False, [3, 3, 3, 3, 3, 3]],
    }

ensamble_in_paletizado2 = {
    'pos1': [False, [1, 2, 3, 4, 5, 6]],
    'pos2': [False, [1, 2, 3, 4, 5, 6]],
    'pos3': [False, [1, 2, 3, 4, 5, 6]],
    'pos4': [False, [1, 2, 3, 4, 5, 6]],
    }

ensamble_in_paletizado3 = {
    'pos1': [False, [1, 2, 3, 4, 5, 6]],
    'pos2': [False, [1, 2, 3, 4, 5, 6]],
    'pos3': [False, [1, 2, 3, 4, 5, 6]],
    'pos4': [False, [1, 2, 3, 4, 5, 6]],
    }

# Conversión de Unidades
conveyor_l = 100 # [mm]
conveyor_l_p = 916 # [px]
px_to_mm = float(conveyor_l/conveyor_l_p)

# Datos del servidor de la cámara
HOST = "192.168.1.126"  # IP Cámara estación 6
#HOST = "127.0.0.5" # Emulador de de cámara estación 6
PORT = 20000  # The port used by the server

def hangle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))
    # TODO：Do different processing according to the error code

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

def move_to_paletizado():
    '''
    Estado en el cual, luego de la revisión, se procede a llevar un ensamble correcto a la posición
    más libre de su respectivo color.
    Luego de juntar 4 ensambles del mismo color, se procede a notificar al operario para que se lleve
    y empaque dichos ensambles
    '''
    if ensamble_in_revision['pos1'][0] == True:
        print('Moviendo ensamble blanco a zona de Paletizado')
        # Mover ensamble blanco al grupo de 4
        if not ensamble_in_paletizado1['pos1'][0]:
            # Mover a la posición 1
            print('Ensamble blanco a pos 1')
            print('Moviendose a: ', ensamble_in_paletizado1['pos1'][1])
            ensamble_in_paletizado1['pos1'][0] = True
        elif not ensamble_in_paletizado1['pos2'][0]:
            print('Ensamble blanco a pos 2')
            print('Moviendose a: ', ensamble_in_paletizado1['pos2'][1])
            ensamble_in_paletizado1['pos2'][0] = True
        elif not ensamble_in_paletizado1['pos3'][0]:
            print('Ensamble blanco a pos 3')
            print('Moviendose a: ', ensamble_in_paletizado1['pos3'][1])
            ensamble_in_paletizado1['pos3'][0] = True
        elif not ensamble_in_paletizado1['pos4'][0]:
            print('Ensamble blanco a pos 4')
            print('Moviendose a: ', ensamble_in_paletizado1['pos4'][1])
            ensamble_in_paletizado1['pos4'][0] = True
            # Mandar mensaje de paletizado completo 
            print('Proceder a embalaje de Ensamble Blanco')
            ensamble_in_paletizado1['pos1'][0] = False
            ensamble_in_paletizado1['pos2'][0] = False
            ensamble_in_paletizado1['pos3'][0] = False
            ensamble_in_paletizado1['pos4'][0] = False
        ensamble_in_revision['pos1'][0] = False
        if ensamble_in_revision['pos2'][0] == True:
            print('Moviendo ensamble azul a zona de Paletizado')
            # Mover ensamble azul al grupo de 4
            if not ensamble_in_paletizado2['pos1'][0]:
                # Mover a la posición 1
                print('Ensamble azul a pos 1')
                print('Moviendose a: ', ensamble_in_paletizado2['pos1'][1])
                ensamble_in_paletizado2['pos1'][0] = True
            elif not ensamble_in_paletizado2['pos2'][0]:
                print('Ensamble azul a pos 2')
                print('Moviendose a: ', ensamble_in_paletizado2['pos2'][1])
                ensamble_in_paletizado2['pos2'][0] = True
            elif not ensamble_in_paletizado1['pos3'][0]:
                print('Ensamble azul a pos 3')
                print('Moviendose a: ', ensamble_in_paletizado2['pos3'][1])
                ensamble_in_paletizado2['pos3'][0] = True
            elif not ensamble_in_paletizado2['pos4'][0]:
                print('Ensamble blanco a pos 4')
                print('Moviendose a: ', ensamble_in_paletizado2['pos4'][1])
                ensamble_in_paletizado2['pos4'][0] = True
                # Mandar mensaje de paletizado completo 
                print('Proceder a embalaje de Ensamble Azul')
                ensamble_in_paletizado2['pos1'][0] = False
                ensamble_in_paletizado2['pos2'][0] = False
                ensamble_in_paletizado2['pos3'][0] = False
                ensamble_in_paletizado2['pos4'][0] = False
            ensamble_in_revision['pos2'][0] = False
        if ensamble_in_revision['pos3'][0] == True:
            print('Moviendo ensamble morado a zona de Paletizado')
            # Mover ensamble morado al grupo de 4
            if not ensamble_in_paletizado3['pos1'][0]:
                print('Ensamble morado a pos 1')
                print('Moviendose a: ', ensamble_in_paletizado3['pos1'][1])
                ensamble_in_paletizado1['pos1'][0] = True
            elif not ensamble_in_paletizado3['pos2'][0]:
                print('Ensamble morado a pos 2')
                print('Moviendose a: ', ensamble_in_paletizado3['pos2'][1])
                ensamble_in_paletizado3['pos2'][0] = True
            elif not ensamble_in_paletizado3['pos3'][0]:
                print('Ensamble morado a pos 3')
                print('Moviendose a: ', ensamble_in_paletizado3['pos3'][1])
                ensamble_in_paletizado3['pos3'][0] = True
            elif not ensamble_in_paletizado3['pos4'][0]:
                print('Ensamble morado a pos 4')
                print('Moviendose a: ', ensamble_in_paletizado3['pos4'][1])
                ensamble_in_paletizado1['pos1'][0] = True
                # Mandar mensaje de paletizado completo 
                print('Proceder a embalaje de Ensamble morado')
                ensamble_in_paletizado3['pos1'][0] = False
                ensamble_in_paletizado3['pos2'][0] = False
                ensamble_in_paletizado3['pos3'][0] = False
                ensamble_in_paletizado3['pos4'][0] = False
            ensamble_in_revision['pos3'][0] = False
        print('to recepción de motores')
        recepcion_ensambles

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

def recepcion_ensambles():
    '''
    Estado en el cual se reciben ensambles de la banda transportadora y son colocados en su base
    correspondiente dependiendo del color de estos ensambles
    Se espera que el color sea detectado por medio de un código de barras, que indique
    tanto si es motor o reductor, su modelo, y su color
    '''
    print('State: Recepcion')
    print('Esperando ensamble en la banda transportadora...')

    # agregar código de recepción de ensambles

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
    ########################################################

main()