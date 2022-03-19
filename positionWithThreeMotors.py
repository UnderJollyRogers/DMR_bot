import canopen
import time
import numpy

network = canopen.Network()
network.connect(bustype='ixxat', channel=0, bitrate=1000000)
joint1 = network.add_node(3, 'od.eds') 
joint2 = network.add_node(2, 'od.eds')
joint3 = network.add_node(1, 'od.eds')
 
# Checking position

pos1 = joint1.sdo[0x6064]
print(pos1.raw)
pos2 = joint2.sdo[0x6064]
print(pos2.raw)
# Setting motor to operation
joint1.sdo[0x6060].raw=1 #Position mode

# node1.sdo[0x6040].raw=0x0000 # Switch on disabled
# node2.sdo[0x6060].raw=1 
# node2.sdo[0x6040].raw=0x0000
# time.sleep(0.5)
# node1.sdo[0x6040].raw=0x0006 # Ready to Switch on
# node2.sdo[0x6040].raw=0x0006
# time.sleep(0.5)
# node1.sdo[0x6040].raw=0x0007 # Switched on
# node2.sdo[0x6040].raw=0x0007
# time.sleep(0.5)
# node1.sdo[0x6040].raw=0x000F # Operation enable
# node2.sdo[0x6040].raw=0x000F
# time.sleep(0.5)

# # Setting motor position 
# P = numpy.linspace(0, 10000,11)
# print(P[0])
# cont = 0
# PA = node1.sdo[0x6064].raw
# CW=node1.sdo[0x6041].raw
# print(CW)
# if PA != 0:
#     node1.sdo[0x607A].raw=0 # Assign Position
#     node2.sdo[0x607A].raw=0
#     node1.sdo[0x6040].raw=0x003F # Refresh
#     node2.sdo[0x6040].raw=0x003F

# CW=node1.sdo[0x6041].raw
# print(CW)
# # time.sleep(5)
# n = len(P)
# while True:
#     pos1 = node1.sdo[0x6064].raw
#     pos2 = node2.sdo[0x6064].raw
#     print(pos1)
#     print(pos2)
#     if P[cont] == pos1 and P[cont] == pos2:
#         print(P[cont])
#         print(pos1)
#         CW=node1.sdo[0x6041].raw
#         print(CW)
#         cont = cont + 1
#         node1.sdo[0x6040].raw=0x000F # Operation enable
#         node2.sdo[0x6040].raw=0x000F
#         node1.sdo[0x607A].raw=P[cont] # Assign Position
#         node2.sdo[0x607A].raw=P[cont]
#         node1.sdo[0x6040].raw=0x003F # Refresh
#         node2.sdo[0x6040].raw=0x003F
        
#     if cont == n-1:
#         break


network.disconnect()
