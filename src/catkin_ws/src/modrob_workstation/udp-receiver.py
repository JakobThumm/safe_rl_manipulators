import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 25000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(512)
    byte_data = bytearray(data)
    print(byte_data)
