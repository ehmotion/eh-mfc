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
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('0.0.0.0', port)
s.bind(server_address)
s.settimeout(10)

print("Ctrl+C to exit the program")

pknt = 0
while True:
  try:
    data, address = s.recvfrom(4096)
    pknt = pknt + 1
    print("received: %d bytes" % len(data))
    #print(' '.join(format(x, '02x') for x in data))
    #print("decoded: %d bytes" % len(ddata))
    #print(' '.join(format(x, '02x') for x in ddata))
    if len(ddata) > 0:
      #https://github.com/Nenkai/PDTools/blob/master/PDTools.SimulatorInterface/SimulatorPacketGT7.cs
      #RPM: 15th 4byte ints
      rpm = struct.unpack('f', ddata[15*4:15*4+4])
      print('RPM %d' %(rpm))
  except Exception as e:
    pass
