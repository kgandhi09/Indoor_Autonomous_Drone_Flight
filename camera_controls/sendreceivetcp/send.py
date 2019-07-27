import socket
from time import sleep

host = '192.168.1.6'
port = 9988

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

comm_flag = True

while True:
    #command = input("Enter command: ")
    #if command == 'KILL':
        #s.send(str.encode(command))
        #break
    if not comm_flag:
        command = "ppm"
        s.send(str.encode(command))
    elif comm_flag:
        comm_flag = False
        reply = s.recv(1024)
        print(reply)

s.close()
