#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 16:21:45 2018

@author: jsousafl
"""

# Echo client program

import struct
import socket
import time
import numpy as np

def circle(center,radius,orientation):
    num_pts = 4
    x = []
    y = []
    z = center[2]
    delta = 0.005
    for t in range(1,num_pts+1):
        x.append(center[0]+radius*np.cos(2*np.pi*t/num_pts))
        y.append(center[1]+radius*np.sin(2*np.pi*t/num_pts))
#    tcp_command0 = " movel(p[%f,%f,%f,%f,%f,%f], a=1.4, v=0.55)\n" % (x[0]+delta,y[0]+delta,z,orientation[0],orientation[1],orientation[2])
#    tcp_command1 = " movep(p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[0],y[0],z,orientation[0],orientation[1],orientation[2],radius)
#    tcp_command2 = " movec(p[%f,%f,%f,%f,%f,%f],p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[1],y[1],z,orientation[0],orientation[1],orientation[2],x[2],y[2],z,orientation[0],orientation[1],orientation[2],radius)
#    tcp_command3 = " movep(p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[2]-delta,y[2]-delta,z,orientation[0],orientation[1],orientation[2],radius)
#    tcp_command4 = " movec(p[%f,%f,%f,%f,%f,%f],p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[3],y[3],z,orientation[0],orientation[1],orientation[2],x[0],y[0],z,orientation[0],orientation[1],orientation[2],radius)
#    s.send(str.encode(tcp_command0))
#    print('commande 0')
#    time.sleep(0.05)
#    s.send(str.encode(tcp_command1))
#    time.sleep(0.05)
#    print('commande 1')
#    s.send(str.encode(tcp_command2))
#    time.sleep(0.05)
#    print('commande 2')
#    s.send(str.encode(tcp_command3))
#    time.sleep(0.05)#5 AQUI FUNCIONA
#    print('commande 3')
#    s.send(str.encode(tcp_command3))
#    time.sleep(0.05)
#    print('commande 3')
#    s.send(str.encode(tcp_command4))
#    print('commande 4')
    #PROTEÇÃO VELOCIDADE
    #tcp_command = " movel(p[%f,%f,%f,%f,%f,%f], a=1.4, v=0.55)\n" % (x[0]+delta,y[0]+delta,z,orientation[0],orientation[1],orientation[2])
    #s.send(str.encode(tcp_command))
    tcp_command = "def process():\n"
    #tcp_command += " set_digital_out(8,False)\n"
    for i in range(3):
        tcp_command += " movep(p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[0],y[0],z,orientation[0],orientation[1],orientation[2],radius)
        tcp_command += " movec(p[%f,%f,%f,%f,%f,%f],p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[1],y[1],z,orientation[0],orientation[1],orientation[2],x[2],y[2],z,orientation[0],orientation[1],orientation[2],radius)
        tcp_command += " movep(p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[2]-delta,y[2]-delta,z,orientation[0],orientation[1],orientation[2],radius)
        # tcp_command += " movep(p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[2]-delta,y[2]-delta,z,orientation[0],orientation[1],orientation[2],radius)
        tcp_command += " movec(p[%f,%f,%f,%f,%f,%f],p[%f,%f,%f,%f,%f,%f],a=1.4,v=0.55,r=%f)\n" % (x[3],y[3],z,orientation[0],orientation[1],orientation[2],x[0],y[0],z,orientation[0],orientation[1],orientation[2],radius)
    #tcp_command += " set_digital_out(8,True)\n"
    tcp_command += "end\n"
    s.send(str.encode(tcp_command))

HOST = "192.168.1.5"   # The remote host
PORT = 30002              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(s.connect((HOST, PORT)))
center = [0.3, 0.15, 0.1]# ANOTAR SOBRE PROTEÇÃO DE VELOCIDADE
orientation = [3.0,-1.0,0.0]
time.sleep(0.05)
tcp_command = " movel(p[%f,%f,%f,%f,%f,%f], a=0.5, v=0.5)\n" % (center[0],center[1],center[2],orientation[0],orientation[1],orientation[2])
#s.send(str.encode(tcp_command))
#time.sleep(0.5)
tcp_command1 = " movej(p[%f,%f,%f,%f,%f,%f], a=1.3962634015954636, v=1.0471975511965976)\n" % (center[0],center[1],center[2],orientation[0],orientation[1],orientation[2])
s.send(str.encode(tcp_command1))

#circle(center,0.05,orientation)
#data = s.recv(1024)
s.close()
#print ("Received", repr(data))


