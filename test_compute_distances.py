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

def import_graph_coll(path):
    #returns a list of lists of graphs 
    graph_coll=[]
    listdir=os.listdir(path)
    for dire in listdir:
        graph_coll.append([])
        graphdir=os.listdir(path+'/'+dire)
        for graph_file in graphdir:
            graph_coll[-1].append(nx.readwrite.gml.read_gml(path+'/'+dire+'/'+graph_file,label='id'))
    return graph_coll

def graph_coll_edit(coll):
    # adds depth data to each vertex in the collection:
    for mR in coll:
        for G in mR:
           for node in G.nodes:
               SP=nx.shortest_path_length(G,0)
               G.nodes[node]['depth']=SP[node]
    return coll
               
def edge_weight(G1,v1_id,G2,v2_id):
    labels_G1 = nx.get_node_attributes(G1, "lbl")
    labels_G2 = nx.get_node_attributes(G2, "lbl")
    if labels_G1[v1_id]!=labels_G2[v2_id]:
        return 0
    all_attr=list(set(list(nx.get_node_attributes(G1,'lbl').values()) +list(nx.get_node_attributes(G2,'lbl').values())))
    num_child_by_attr_v1=np.zeros((len(all_attr)))
    num_child_by_attr_v2=np.zeros((len(all_attr)))
    for child in G1.neighbors(v1_id):
        for i in range(len(all_attr)):
            if labels_G1[child]==all_attr[i]:
                num_child_by_attr_v1[i]+=1
    for child in G2.neighbors(v2_id):
        for i in range(len(all_attr)):
            if labels_G2[child]==all_attr[i]:
                num_child_by_attr_v2[i]+=1
    score=0.00001
    for i in range(len(all_attr)):
        score+=min([num_child_by_attr_v1[i],num_child_by_attr_v2[i]])
    return score

def create_LM_graph(G1,G2):
    indices_G1=list(range(G1.order()))
    indices_G2=list(range(G1.order(),G1.order()+G2.order()))
    LMG=nx.Graph()
    LMG.add_nodes_from(indices_G1)
    LMG.add_nodes_from(indices_G2)
    for v in range(G1.order()):
        LMG.nodes[v]['depth']=G1.nodes[v]['depth']
    for v in range(G2.order()):
        LMG.nodes[indices_G2[v]]['depth']=G2.nodes[v]['depth']
    for v1 in range(G1.order()):
        for v2 in range(G2.order()):
            LMG.add_edge(v1,indices_G2[v2],weight=edge_weight(G1,v1,G2,v2))
    return LMG
            
def approx_alg(G1,G2):
    LMG=create_LM_graph(G1,G2)
    indices_G1=list(range(G1.order()))
    indices_G2=list(range(G1.order(),G1.order()+G2.order()))
    V1_even=[]
    V1_odd=[]
    V2_even=[]
    V2_odd=[]
    for i,node in enumerate(LMG.nodes):
        if LMG.nodes[node]['depth']%2==0:
            if i<G1.order():
                V1_even.append(i)
            else:
                V2_even.append(i)
        elif i<G1.order():
            V1_odd.append(i)
        else:
            V2_odd.append(i)
    Vs=[]
    Vs.append(V1_even+V2_even)
    Vs.append(V1_even+V2_odd)
    Vs.append(V1_odd+V2_even)
    Vs.append(V1_odd+V2_odd)
    Sols=[]
    for V in Vs:
        LMG_temp=LMG.subgraph(V)
        Sols.append(list(nx.max_weight_matching(LMG_temp)))
    return Sols
    
        
                
        
graph_coll=import_graph_coll(path)    
graph_coll=graph_coll_edit(graph_coll)
Sols=approx_alg(graph_coll[0][0],graph_coll[4][0])
                


    
#graph_coll=[]
#for dire in lis_dir
