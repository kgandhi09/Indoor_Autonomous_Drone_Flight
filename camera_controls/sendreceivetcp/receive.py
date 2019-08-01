import socket
import time

host = '192.168.0.100'
port = 9988

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

conn,address=0,0
def setupServer():
    global s
    print("Socket created.")
    try:
        s.bind((host, port))
        print("bind")
    except socket.error as msg:
        print(msg)
    print("Socket bind complete.")
    return s

def setupConnection():
    global conn,address
    setupServer()
    s.listen(1) # Allows one connection at a time.
    print("k")
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn

def dataTransfer(conn):
    while True:
        data = conn.recv(1024) # receive the data
        print(data)
        data = data.decode('utf-8')
        if data == 'kill':
            print("Server is shutting down.")
            s.close()
            break
        elif data == 'test':
            reply ="1"
        else:
            reply = b'Unknown Command'
        conn.sendall(reply)
    conn.close()

#setupServer()
setupConnection()
dataTransfer(conn)
