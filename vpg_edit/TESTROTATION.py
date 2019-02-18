#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  5 14:56:31 2019

@author: ecl
"""
import numpy as np
import utils

theta = 41.8*np.pi/180

coord_init_x_y_z = np.transpose(np.array([0.432,0.129,0.05]))
coord_init_u_v_z = np.transpose(np.array([-0.24144356,0.1530163,0.041]))

R = utils.euler2rotm([0,0,-theta])
Rprime = utils.euler2rotm([0,0,theta])

#print(R)
#print('oi')
#print(Rprime)

coord_fini_u_v_z = np.dot(Rprime,coord_init_x_y_z)
coord_fini_x_y_z = np.dot(R,coord_init_u_v_z)


print(coord_fini_u_v_z)
#print(coord_fini_x_y_z)


def cart2altern(coord_pos): #vector must be column
    theta = 41.8*np.pi/180
    Rprime = utils.euler2rotm([0,0,theta])
    coord_final_u_v_z = np.dot(Rprime,coord_pos)
    return coord_final_u_v_z

def altern2cart(coord_pos): #vector must be column
    theta = 41.8*np.pi/180
    R = utils.euler2rotm([0,0,-theta])
    coord_final_x_y_z = np.dot(R,coord_pos)
    return coord_final_x_y_z
    