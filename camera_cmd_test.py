import socket
import time

# Datos del servidor de la cámara
HOST = "192.168.1.126"  # Emulador
#HOST = "127.0.0.5" # Cámara estación 6
PORT = 20000  # The port used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
s.connect((HOST, PORT))



send_com = "cmd trigger"

s.send(send_com.encode())
            
        

