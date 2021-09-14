import struct
import socket
import time
import math
from twisted.internet import task, reactor

timeout = 0.001 # 1 ms
UDP_IP_OUT = "127.0.0.1"
UDP_PORT_OUT = 25000
UDP_IP_IN = "127.0.0.1"
UDP_PORT_IN = 25001
ANGLE_MODE = 0x04;
TORQUE_MODE = 0x05;
MODE = ANGLE_MODE;

print ("UDP IP OUT:", UDP_IP_OUT)
print ("UDP PORT OUT:", UDP_PORT_OUT)

out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
out_sock.bind((UDP_IP_IN, UDP_PORT_IN))

q_start = [0, 0, 0]

dof = 3

print ("DOF found: ", dof)

#start_milis = time.time_ns() / 1000 / 1000 / 1000;

def send_config_measured_command():
	CONFIG_MEASURED_MESSAGE = bytearray()

	CONFIG_MEASURED_MESSAGE.append(0x05)

	payload_length = dof * 36 + 2;
	CONFIG_MEASURED_MESSAGE.extend(struct.pack('H', payload_length))

	sequence_number = 1;
	#milis = time.time_ns() / 1000 / 1000 / 1000;

	# Robot state
	CONFIG_MEASURED_MESSAGE.extend([0x04])

	# Tool Activation
	CONFIG_MEASURED_MESSAGE.extend([0x01])

	frequency = timeout * 100
	amplitude = 0.1

	for i in range(1, dof+1):

		CONFIG_MEASURED_MESSAGE.extend(struct.pack("d", 1))
		CONFIG_MEASURED_MESSAGE.extend(struct.pack("d", 2))
		CONFIG_MEASURED_MESSAGE.extend(struct.pack("d", 3))
		CONFIG_MEASURED_MESSAGE.extend(struct.pack("d", 4))
		CONFIG_MEASURED_MESSAGE.extend(struct.pack("d", 5))

	out_sock.sendto(CONFIG_MEASURED_MESSAGE, (UDP_IP_OUT, UDP_PORT_OUT))


time.sleep(1)

l = task.LoopingCall(send_config_measured_command)
l.start(timeout)

reactor.run()
