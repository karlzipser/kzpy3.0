print "d.py server side"

from kzpy3.utils import *
import socket

host = '127.0.0.1' # 'localhost'
port = 5001

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind((host, port))
serversocket.listen(5) # become a server socket, maximum 5 connections

connection, address = serversocket.accept()
while True:
    buf = connection.recv(64)
    if len(buf) > 0:
        try:
            t = eval(buf)
        except:
            t = (0,0)
        print(d2s(t[0],t[1]))
        #break
        if buf == 'q':
            time.sleep(0.1)
            break

serversocket.close()



"""
import socket

def netcat(hostname, port, content):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((hostname, port))
    s.sendall(content)
    s.shutdown(socket.SHUT_WR)
    while 1:
        data = s.recv(1024)
        if data == "":
            break
        print "Received:", repr(data)
    print "Connection closed."
    s.close()
"""