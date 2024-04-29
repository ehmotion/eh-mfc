import socket
import sys
import struct
# pip3 install salsa20
# from salsa20 import Salsa20_xor

#https://github.com/Nenkai/PDTools/blob/master/SimulatorInterface/SimulatorInterface.cs
SendDelaySeconds = 10

port = 20777

if len(sys.argv) == 2:
    # Get "IP address of Server" and also the "port number" from
    ip = sys.argv[1]
else:
    print("Run like : python3 gt7racedata.py <playstation-ip>")
    exit(1)

# Create a UDP socket
r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
rcv_address = ('127.0.0.1', port)
r.bind(rcv_address)
#r.settimeout(10)
snd_address = ('0.0.0.0', port+1)
s.bind(snd_address)
#s.settimeout(10)

print("Ctrl+C to exit the program")

def send_data(s, data, sip, sport):
  #send HB
  s.sendto(data, (sip, sport))
  print('sent data ' + str(len(data)) + 'B to ' + sip + ':' + str(sport))

print('listen for data on ' + str(rcv_address))

pknt = 0
while True:
  try:
    data, address = s.recvfrom(4096)
    pknt = pknt + 1
    print("received: %d bytes" % len(data))
    send_data(s, data, port)
    #print(' '.join(format(x, '02x') for x in data))
    #print("decoded: %d bytes" % len(ddata))
    #print(' '.join(format(x, '02x') for x in ddata))
  except Exception as e:
    pass
