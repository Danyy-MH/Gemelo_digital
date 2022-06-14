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
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI



# Definición de variables

robot_is_on = True
variables = {'move_robot': 0, 'contador': 0}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}
camera_data = [0, 0, 0] 
x_center = 640
y_center = 512

x_start = -68.3
y_start = 334.8
cam_offset = 90
pick_motor = [0 ,0, 0]
angle = 0
x_comp = 0
y_comp = 0

# Conversión de Unidades
conveyor_l = 100 # [mm]
conveyor_l_p = 916 # [px]
px_to_mm = float(conveyor_l/conveyor_l_p)

# Datos del servidor de la cámara
HOST = "192.168.1.126"  # Emulador
#HOST = "127.0.0.5" # Cámara estación 6
PORT = 20000  # The port used by the server

#######################################################
"""
Just for test example
"""
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
        print('xArm ip address : ', ip)
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################

def hangle_err_warn_changed(item):
    print('ErrorCode: {}, WarnCode: {}'.format(item['error_code'], item['warn_code']))
    # TODO：Do different processing according to the error code

def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
    except:
        print(*args, **kwargs)

def get_data_from_camera():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(10)
        try:
            s.connect((HOST, PORT))
            data = s.recv(1024)
        except:
            print("No se recibió data de la cámara")

    # String manipulation
    data_str = str(data)
    data_str = data_str.split(";")
    #print(data_str)

    x_motor = data_str[0]
    x_motor = float(x_motor[-3:])

    y_motor = float(data_str[1])

    angle_ref = float(data_str[2])
    # Cálculo a mm con respecto al sensor foto-eléctrico
    x_motor = x_center - x_motor
    y_motor = y_center - y_motor


    # #Cálculo de ángulo (solo primer cuadrante)
    if angle_ref >= 0 and angle_ref <= 90:
        angle = 90 - angle_ref
        x_comp = 0
        y_comp = 0
    elif angle_ref < 0 and angle_ref >= -90:
        angle = abs(angle_ref) + 90
        x_comp = 0 #7
        y_comp = 0 #-1
        
        



    # comp_x = 7*math.sin(abs(angle))
    
    # comp_y = 7*math.cos(abs(angle)) # + 4
    

    x_mm = x_motor*px_to_mm
    y_mm = y_motor*px_to_mm

    print('Posición en x: ', x_mm)
    print('Posición en y: ', y_mm)
    print('Ángulo: ', angle)




    return x_mm, y_mm, angle, x_comp, y_comp
arm = XArmAPI('192.168.1.206', do_not_open=True)
arm.register_error_warn_changed_callback(hangle_err_warn_changed)
arm.connect()

# enable motion
arm.motion_enable(enable=True)
# set mode: position control mode
arm.set_mode(0)
# set state: sport state
arm.set_state(state=0)

print('=' * 50)
print('version:', arm.get_version())
print('state:', arm.get_state())
print('cmdnum:', arm.get_cmdnum())
print('err_warn_code:', arm.get_err_warn_code())
print('position(°):', arm.get_position(is_radian=False))
print('position(radian):', arm.get_position(is_radian=True))
print('angles(°):', arm.get_servo_angle(is_radian=False))
print('angles(radian):', arm.get_servo_angle(is_radian=True))
print('angles(°)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=False))
print('angles(radian)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=True))
print('=' * 50)

##################################################

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
while True:
    if params['quit']:
        break
    if arm.get_cgpio_digital(0)[1] == 0 and params['variables'].get('move_robot', 0) == True:
        if arm.error_code == 0 and not params['quit']:
            arm.set_pause_time(1)
        # if arm.error_code == 0 and not params['quit']:
        #     code = arm.set_servo_angle(angle=[0.0, -24.6, -33.1, 0.0, 60.2, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
        #     if code != 0:
        #         params['quit'] = True
        #         pprint('set_servo_angle, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[-68.3, 334.8, 434.4, -180, 0, 90], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_cgpio_digital(5, 1, delay_sec=0)
            if code != 0:
                params['quit'] = True
                pprint('set_cgpio_digital, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            arm.set_pause_time(2)
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_cgpio_digital(5, 0, delay_sec=0)
            if code != 0:
                params['quit'] = True
                pprint('set_cgpio_digital, code={}'.format(code))
        ### Take photo and receive data via TCP
        camera_data = get_data_from_camera()
        pick_motor[0] = x_start + camera_data[0] + camera_data[3]
        pick_motor[1] = y_start + cam_offset + camera_data[1] + camera_data[4]
        pick_motor[2] = camera_data[2] 
        print(camera_data)
        print(pick_motor)
        

        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[-68.3, pick_motor[1], 434.4, -180, 0, 90], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[pick_motor[0], pick_motor[1], 434.4, -180, 0, 90], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[pick_motor[0], pick_motor[1], 434.4, -180, 0, pick_motor[2]], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[pick_motor[0], pick_motor[1], 250, -180, 0, pick_motor[2]], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(*[pick_motor[0], pick_motor[1], 230, -180, 0, pick_motor[2]], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        # states = arm.get_joint_states(num=3)
        # print("joint 3 states:" + str(states[0]))
        # arm.disconnect()
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
            code = arm.set_position(*[pick_motor[0], pick_motor[1], 300, -179.7, 1.2, pick_motor[2]], speed=params['speed'], mvacc=params['acc'], radius=-1.0, wait=True)
            if code != 0:
                params['quit'] = True
                pprint('set_position, code={}'.format(code))
        if arm.error_code == 0 and not params['quit']:
            arm.set_pause_time(1)

        ##########################################

        if params['variables'].get('contador', 0) == 0:
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -37.5, -34.2, 0.0, 73.3, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-32.8, -19.2, -22.3, -0.3, 42.1, 13.4], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-31.6, -9, -22.8, -0.3, 34.3, 11.5], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_cgpio_digital(0, 0, delay_sec=0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_cgpio_digital, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                arm.set_pause_time(1)
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-32.8, -19.2, -22.3, -0.3, 42.1, 13.4], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -24.6, -33.1, 0.0, 60.2, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if not params['quit']:
                params['variables']['move_robot'] = False

        #########################################

        elif params['variables'].get('contador', 0) == 1:
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -37.5, -34.2, 0.0, 73.3, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-16.9, -16.0, -25.7, 0.1, 42.3, -19.7], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-16, -7.2, -22.7, 0.2, 28.9, 33.4], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_cgpio_digital(0, 0, delay_sec=0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_cgpio_digital, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                arm.set_pause_time(1)
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-16.9, -16.0, -25.7, 0.1, 42.3, -19.7], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -24.6, -33.1, 0.0, 60.2, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if not params['quit']:
                params['variables']['move_robot'] = False
        elif params['variables'].get('contador', 0) == 2:
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -37.5, -34.2, 0.0, 73.3, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-24.4, -42.2, -8.2, -0.1, 51.0, 18], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-22.9, -27.5, -4.6, -0.1, 32.7, 19.4], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_cgpio_digital(0, 0, delay_sec=0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_cgpio_digital, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                arm.set_pause_time(1)
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[-24.4, -42.2, -8.2, -0.1, 51.0, -27.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if arm.error_code == 0 and not params['quit']:
                code = arm.set_servo_angle(angle=[0.0, -24.6, -33.1, 0.0, 60.2, -0.1], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
                if code != 0:
                    params['quit'] = True
                    pprint('set_servo_angle, code={}'.format(code))
            if not params['quit']:
                params['variables']['move_robot'] = False
        if not params['quit']:
            params['variables']['contador'] += 1
        if params['variables'].get('contador', 0) == 3:
            break
    elif arm.get_cgpio_digital(0)[1] == 1:
        if not params['quit']:
            params['variables']['move_robot'] = True
            pick_motor[0] = 0
            pick_motor[1] = 0
            pick_motor[2] = 0
if arm.error_code == 0 and not params['quit']:
    code = arm.set_servo_angle(angle=[37.8, 41.5, -75.2, 40.6, 109.8, -67.6], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
    if code != 0:
        params['quit'] = True
        pprint('set_servo_angle, code={}'.format(code))
if arm.error_code == 0 and not params['quit']:
    code = arm.set_servo_angle(angle=[-5.6, -17.6, -105.5, 1.7, 123.2, 85.2], speed=params['angle_speed'], mvacc=params['angle_acc'], wait=True, radius=-1.0)
    if code != 0:
        params['quit'] = True
        pprint('set_servo_angle, code={}'.format(code))  




###################################################

print('=' * 50)
code, states = arm.get_cgpio_state()
print('get_cgpio_state, code={}'.format(code))
print('GPIO state: {}'.format(states[0]))
print('GPIO error code: {}'.format(states[1]))
print('Digital->Input->FunctionalIO: {}'.format([states[2] >> i & 0x0001 for i in range(8)]))
print('Digital->Input->ConfiguringIO: {}'.format([states[3] >> i & 0x0001 for i in range(8)]))
print('Digital->Output->FunctionalIO: {}'.format([states[4] >> i & 0x0001 for i in range(8)]))
print('Digital->Output->ConfiguringIO: {}'.format([states[5] >> i & 0x0001 for i in range(8)]))
print('Analog->Input: {}'.format(states[6:8]))
print('Analog->Output: {}'.format(states[8:10]))
print('Digital->Input->Conf: {}'.format(states[10]))
print('Digital->Output->Conf: {}'.format(states[11]))

arm.disconnect()