import socket
HOST = "192.168.1.138" 
PORT = 63352
robotiq = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
robotiq.connect((HOST,PORT))
robotiq.sendall(b'SET ACT 1\n') 
# robotiq.sendall(b'SET GTO 0\n')
# robotiq.sendall(b'SET MOD 1\n')
# robotiq.sendall(b'SET POS 0\n')
# robotiq.sendall(b'SET POS 255\n')
# robotiq.sendall(b'SET GTO 1\n')
def gripperOFF():
    robotiq.sendall(b'SET ACT 1\n') 
    robotiq.sendall(b'SET GTO 0\n')
    robotiq.sendall(b'SET MOD 1\n')
    robotiq.sendall(b'SET POS 0\n')
    robotiq.sendall(b'SET GTO 1\n')
def gripperOn():
    robotiq.sendall(b'SET ACT 1\n')
    robotiq.sendall(b'SET GTO 0\n')
    robotiq.sendall(b'SET MOD 1\n')
    robotiq.sendall(b'SET POS 255\n')
    robotiq.sendall(b'SET GTO 1\n')
def getPosition():
    robotiq.sendall(b'GET POS\n')
    data = s.recv(2**10) 
    print("",data)
print("sucess")
# getPosition()