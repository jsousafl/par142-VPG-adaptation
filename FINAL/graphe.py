#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 12:08:59 2019

@author: jsousafl
"""

import networkx as nx
import numpy as np


class Graphe(object):
    def __init__(self, matlab_data, matrix_adj, pas, matrix_nodes,nb_pts,u_limits,v_limits,z_limits,u_forb_limits,v_forb_limits,z_forb_limits):
        
        self.nb_pts = nb_pts
        self.step = pas
        #Adjacency matrix
        self.np_ADJ = np.array(matrix_adj)
        self.nx_ADJ_matrix = nx.from_numpy_matrix(self.np_ADJ, create_using=nx.DiGraph())
        
        #Coordinates of the points
        self.u_discrete = np.linspace(u_limits[0],u_limits[1],self.nb_pts)
        self.v_discrete = np.linspace(v_limits[0],v_limits[1],self.nb_pts)
        self.z_discrete = np.linspace(z_limits[0],z_limits[1],self.nb_pts)        
        self.points_3D = np.vstack(np.meshgrid(self.z_discrete,self.v_discrete,self.u_discrete)).reshape(3,-1).T
        
        self.u_limits = u_limits
        self.v_limits = v_limits
        self.z_limits = z_limits
        
        self.heightmap_resolution = 0.00175
        self.workspace_limits = np.asarray([[-0.112, 0.112], [0.16, 0.384], [0.0, 0.12]])
        
        self.save_points_3D()
        self.defining_forbidden_zone_robot(u_forb_limits,v_forb_limits,z_forb_limits)
    
    def save_points_3D(self):
        np.savetxt('points_3D.txt',self.points_3D,fmt="%.4f")
    def finding_minimal_path(self,initial_node,final_node): #tested
        node_list = nx.dijkstra_path(self.nx_ADJ_matrix, initial_node, final_node)
        return node_list
    def find_closest_node(self,coord_node):#tested
        dist_of_all = np.array([])
        coord_node = np.round(coord_node,5)
        for i in range(self.points_3D.shape[0]):
            aux = np.round(np.array(np.copy(self.points_3D[i,:]),dtype=float),5)
            aux.shape = (1,3)
            dist_of_all = np.insert(dist_of_all,dist_of_all.shape[0],np.linalg.norm(coord_node-aux), 0)
        closest_node = np.argmin(dist_of_all)
        return closest_node
        
    def associate_coord2node(self,node):#tested
        coord_uvz_point = np.flip(self.points_3D[node,:])
        return coord_uvz_point
    def associate_coord2nodelist(self,node_list):#tested
        coord_uvz_list = np.array([])
        for node in node_list:
            coord_uvz_list = np.insert(coord_uvz_list, coord_uvz_list.shape[0],self.associate_coord2node(node), 0).reshape(coord_uvz_list.shape[0]+1,3)
        return coord_uvz_list
    def associate_node2coord(self,coord_uvz):#tested
        coord_uvz.shape = (1,3)
        coord_uvz = np.flip(coord_uvz,axis=1)
        correspond_node = self.find_closest_node(coord_uvz)
        return correspond_node
    def defining_forbidden_zone_robot(self,u_forbidden_zone,v_forbidden_zone,z_forbidden_zone):
        #u_forbidden_zone -> u min max
        #v_forbidden_zone -> v min max
        vertex1 = np.array([u_forbidden_zone[0],v_forbidden_zone[0],0.0])
        vertex2 = np.array([u_forbidden_zone[1],v_forbidden_zone[0],0.0])
        vertex4 = np.array([u_forbidden_zone[1],v_forbidden_zone[1],0.0])
        node_vertex_1 = self.associate_node2coord(vertex1)
        node_vertex_2 = self.associate_node2coord(vertex2)
        node_vertex_4 = self.associate_node2coord(vertex4)
#        print("--------------------------------------------")
#        print(node_vertex_1)
#        print(node_vertex_2)
#        print(node_vertex_4)
        num_intervals_u = (node_vertex_2-node_vertex_1)
        num_intervals_v = np.round((node_vertex_4-node_vertex_2)/(self.nb_pts*self.nb_pts))
        edge_1_2 = np.linspace(node_vertex_1,node_vertex_2,num_intervals_u+1)
        forbidden_plane = np.array([])
        for vertex in edge_1_2:
            new_edge = np.linspace(vertex,vertex+(num_intervals_v*(self.nb_pts*self.nb_pts)),num_intervals_v+1)
            new_edge = new_edge.reshape(1,new_edge.shape[0])
#            print(new_edge)
            forbidden_plane = np.insert(forbidden_plane, forbidden_plane.shape[0],new_edge[0], 0)
        forbidden_region = np.array([])       
        for i in range(int(self.nb_pts)):
            new_plane = forbidden_plane+(i*self.nb_pts)
            new_plane = new_plane.reshape(1,new_plane.shape[0])
            forbidden_region = np.insert(forbidden_region,forbidden_region.shape[0],new_plane[0],0)
        forbbiden_nodes = np.sort(forbidden_region)
#        print(forbidden_region)
        for node in forbbiden_nodes:
#            print(int(node))
            self.np_ADJ[int(node),:] = 0
            self.np_ADJ[:,int(node)] = 0
#            print(self.np_ADJ[int(node),:].shape[0])
        self.nx_ADJ_matrix = nx.from_numpy_matrix(self.np_ADJ, create_using=nx.DiGraph())
        
        #Managing height problem
        vertex1 = np.array([self.u_limits[0],self.v_limits[0],z_forbidden_zone[0]])
        vertex2 = np.array([self.u_limits[1],self.v_limits[0],z_forbidden_zone[0]])
        vertex4 = np.array([self.u_limits[1],self.v_limits[1],z_forbidden_zone[0]])
        vertex5 = np.array([self.u_limits[1],self.v_limits[1],z_forbidden_zone[1]])
        node_vertex_1 = self.associate_node2coord(vertex1)
        node_vertex_2 = self.associate_node2coord(vertex2)
        node_vertex_4 = self.associate_node2coord(vertex4)
        node_vertex_5 = self.associate_node2coord(vertex5)
        num_intervals_u = (node_vertex_2-node_vertex_1)
        num_intervals_v = np.round((node_vertex_4-node_vertex_2)/(self.nb_pts*self.nb_pts))
        edge_1_2 = np.linspace(node_vertex_1,node_vertex_2,num_intervals_u+1)
        forbidden_plane = np.array([])
        for vertex in edge_1_2:
            new_edge = np.linspace(vertex,vertex+(num_intervals_v*(self.nb_pts*self.nb_pts)),num_intervals_v+1)
            new_edge = new_edge.reshape(1,new_edge.shape[0])
#            print(new_edge)
            forbidden_plane = np.insert(forbidden_plane, forbidden_plane.shape[0],new_edge[0], 0)
        forbidden_region = np.array([]) 
        num_intervals_z = np.round((node_vertex_5 - node_vertex_4)/self.nb_pts)
        for i in range(int(num_intervals_z+1)):
            new_plane = forbidden_plane+(i*self.nb_pts)
            new_plane = new_plane.reshape(1,new_plane.shape[0])
            forbidden_region = np.insert(forbidden_region,forbidden_region.shape[0],new_plane[0],0)
        forbbiden_nodes = np.sort(forbidden_region)
        for node in forbbiden_nodes:
#            print(int(node))
            self.np_ADJ[int(node),:] = 0
            self.np_ADJ[:,int(node)] = 0
#            print(self.np_ADJ[int(node),:].shape[0])
        self.nx_ADJ_matrix = nx.from_numpy_matrix(self.np_ADJ, create_using=nx.DiGraph())
    def defining_obstacles(self,valid_dh):
        y,x = np.where(valid_dh > 0.03)
        u_list = np.array([])
        v_list = np.array([])
        z_list = np.array([])
        for i in range(y.shape[0]):
          target_u = self.workspace_limits[0][1] - (x[i]-48)*self.heightmap_resolution
          target_v = self.workspace_limits[1][0] + (y[i]-30)*self.heightmap_resolution
          target_z = valid_dh[y[i]][x[i]]
          u_list = np.insert(u_list,u_list.shape[0],target_u,0)
          v_list = np.insert(v_list,v_list.shape[0],target_v,0)
          z_list = np.insert(z_list,z_list.shape[0],target_z,0)
        u_list = u_list.reshape(u_list.shape[0],1)
        v_list = v_list.reshape(v_list.shape[0],1)
        z_list = z_list.reshape(z_list.shape[0],1)  
#        if not(u_list.shape[0]==v_list.shape[0]==z_list.shape[0]):
#            print("Invalid list")
#            return
#        else:
#            u_list = u_list.reshape(u_list.shape[0],1)
#            v_list = v_list.reshape(v_list.shape[0],1)
#            z_list = z_list.reshape(z_list.shape[0],1)
        coord_list = np.concatenate((u_list,v_list,z_list),axis=1)
        vertex_list = np.array([])
        for coord_u_v_z in coord_list:
            new_vertex = self.associate_node2coord(coord_u_v_z)
            vertex_list = np.insert(vertex_list,vertex_list.shape[0],new_vertex,0)
#        return(vertex_list)
        vertex_list = np.unique(vertex_list)
        for node in vertex_list:
#            print(int(node))
            self.np_ADJ[int(node),:] = 0
            self.np_ADJ[:,int(node)] = 0
#            print(self.np_ADJ[int(node),:].shape[0])
        self.nx_ADJ_matrix = nx.from_numpy_matrix(self.np_ADJ, create_using=nx.DiGraph())
          
            