#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 18:20:28 2021

@author: nboria
"""

import import_trees as im
import os
import networkx as nx
import numpy as np

path="/home/nboria/Documents/code_duos/data"

outpath="/home/nboria/Documents/code_duos/res"

graph_coll=[]
listdir=os.listdir(path)
for dire in listdir:
    graph_coll.append([])
    graphdir=os.listdir(path+'/'+dire)
    for graph_file in graphdir:
        graph_coll[-1].append(nx.readwrite.gml.read_gml(path+'/'+dire+'/'+graph_file,label='id'))
        

def graph_coll_edit(coll):
    # adds depth data to each vertex in the collection:
    for mR in coll:
        for G in mR:
           for node in G.nodes:
               G.nodes[node]['depth']=nx.shortest_path_length(G,0)
               
def edge_weight(G1,v1_id,G2,v2_id):
    if G1[v1_id]['lbl']!=G2[v2_id]['lbl']:
        return 0
    all_attr=list(set(nx.get_node_attributes(G1,'lbl')+nx.get_node_attributes(G2,'lbl')))
    num_child_by_attr_v1=np.zeroes((len(all_attr)))
    num_child_by_attr_v2=np.zeroes((len(all_attr)))
    for child in G1.neighbors(v1_id):
        for i in range(len(all_attr)):
            if G1[child]['lbl']==all_attr[i]:
                num_child_by_attr_v1[i]+=1
    for child in G2.neighbors(v1_id):
        for i in range(len(all_attr)):
            if G2[child]['lbl']==all_attr[i]:
                num_child_by_attr_v2[i]+=1
    score=0
    for i in range(len(all_attr)):
        score+=min([num_child_by_attr_v1[i],num_child_by_attr_v2[i]])
    return score
                
        
    
graph_coll_edit(graph_coll)
                


    
#graph_coll=[]
#for dire in lis_dir
