#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 18:20:28 2021

@author: nboria
"""

import import_trees as im
import gurobipy as gu
import os
import networkx as nx
import numpy as np
import time

path="/home/nboria/Documents/code_duos/data"

outpath="/home/nboria/Documents/code_duos/res"

def intersection(lst1, lst2):
    lst3 = [value for value in lst1 if value in lst2]
    return lst3

class Evaluator:
    def __init__(self):
        self._preserved_duos_G1=[]
        self._preserved_duos_G2=[]
        self._evaluation=0
        return None
        
    
    def evaluate_sol(self,G1,G2,sol):
        self._preserved_duos_G1=[]
        self._preserved_duos_G2=[]
        self._evaluation=0
        matched_G1=[m[0] for m in sol]
        #print('matched_G1',matched_G1)
        matched_G2=[m[1]-G1.order() for m in sol]
        #print('matched_G2',matched_G2)
        edges_G1=list(G1.edges)
        edges_G2=list(G2.edges)
        #print(edges_G1)
        for i in range(len(matched_G1)):
            for j in range(len(matched_G1)):
                if (matched_G1[i],matched_G1[j]) in edges_G1:
                    if (matched_G2[i],matched_G2[j]) in edges_G2:
                        self._evaluation+=1
                        self._preserved_duos_G1.append([matched_G1[i],matched_G1[j]])
                        self._preserved_duos_G2.append([matched_G2[i],matched_G2[j]])                        
        return self._evaluation
        
class Gurobi_solver:
    def __init__(self,verbose=None):
        self._name='GUROBI'
        self._sol=None
        self._verbose=verbose
        if self._verbose is None:
            self._verbose=False
        self._preserved_duos_G1=[]
    
    def compute_distance(self,G1,G2):
        self._preserved_duos_G1=[]
        self._sol=[]
        m=gu.Model('distance')
        m.setParam('OutputFlag', self._verbose)
        
        #VARIABLES
        x = {}
        # xij = mapping node i to node j
        for i in range(G1.order()):
            for j in range(G2.order()):
                x[i, j] = m.addVar(vtype=gu.GRB.BINARY)
        y = {}
        # yij =1 if duo ij is saved else 0
        for edge in list(G1.edges):
            y[edge[0],edge[1]]=m.addVar(vtype=gu.GRB.BINARY)
        
        m.update()
        
        #CONSTRAINTS
        
        for i in range(G1.order()):
            for j in range(G2.order()):
                if G1.nodes[i]['lbl']!= G2.nodes[j]['lbl']:
                    m.addConstr(x[i, j]==0)
        for i in range(G1.order()):
            m.addConstr(gu.quicksum(x[i,j] for j in range(G2.order()))<=1)
        for j in range(G2.order()):
            m.addConstr(gu.quicksum(x[i,j] for i in range(G1.order()))<=1)
        for edge1 in list(G1.edges):
            m.addConstr(y[edge1[0],edge1[1]]<=gu.quicksum(x[edge1[0],edge2[0]]*x[edge1[1],edge2[1]] for edge2 in list(G2.edges)))
        
        #OBJECTIVE
        m.setObjective(gu.quicksum(y[edge[0],edge[1]] for edge in list(G1.edges)), gu.GRB.MAXIMIZE)
        
        m.optimize()
        self._sol=[]
        for i in range(G1.order()):
            for j in range(G2.order()):
                if x[i,j].X>=0.99999:
                    self._sol.append([i,j+G1.order()])
        
        for edge in list(G1.edges):
            if y[edge[0],edge[1]].X>=0.99999:
                self._preserved_duos_G1.append([edge[0],edge[1]])
 
class Approx_alg:
    def __init__(self):
        self._name='4-APPROX'
        return None

    def compute_distance(self,G1,G2):
        self.init_match(G1,G2)
        self.complete_Sols(G1,G2)
        self._best_eval=0
        E=Evaluator()
        for sol in self._sols:
            if E.evaluate_sol(G1,G2,sol)>= self._best_eval:
                self._best_eval=E._evaluation
                self._sol=sol
                
            

    def edge_weight(self,G1,v1_id,G2,v2_id):
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
    
    def create_LM_graph(self,G1,G2):
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
                LMG.add_edge(v1,indices_G2[v2],weight=self.edge_weight(G1,v1,G2,v2))
        return LMG
    
    def complete_Sols(self,G1,G2):
        Sols=self._sols
        indices_G1=list(range(G1.order()))
        indices_G2=list(range(G1.order(),G1.order()+G2.order()))
        for s,Sol in enumerate(Sols):
            unmatched=indices_G1+indices_G2
            matched=[]
            for edge in Sol:
                matched.append(edge[0])
                matched.append(edge[1])
                unmatched.remove(edge[0])
                unmatched.remove(edge[1])
            RG=nx.Graph()
            RG.add_nodes_from(unmatched)
            for v1 in intersection(indices_G1,unmatched):
                for v2 in intersection(indices_G2,unmatched):
                    if G1.nodes[v1]['lbl']!=G2.nodes[v2-G1.order()]['lbl']:
                        edge_weight=0
                    else:
                        edge_weight=0.000001
                        if len(list(G1.predecessors(v1)))==len(list(G2.predecessors(v2-G1.order())))==1:
                            f_v1=list(G1.predecessors(v1))[0]
                            f_v2=list(G2.predecessors(v2-G1.order()))[0]
                            if [f_v1,indices_G2[f_v2]] in Sol:
                                edge_weight+=1
                        matched_sons_v1=intersection(matched,G1.neighbors(v1))
                        matched_sons_v2=intersection(matched,G2.neighbors(v2-G1.order()))
                        for s_v1 in matched_sons_v1:
                            for s_v2 in matched_sons_v2:
                                if [s_v1,indices_G2[s_v2]] in Sol:
                                    edge_weight+=1
                    RG.add_edge(v1,v2,weight=edge_weight)
            sol_add=list(nx.max_weight_matching(RG))
            for i,match in enumerate(sol_add):
                sol_add[i]=sorted(match)
            Sols[s]=Sol+sol_add
        self._sols=Sols
            
                
    def init_match(self,G1,G2):
        LMG=self.create_LM_graph(G1,G2)
        #indices_G1=list(range(G1.order()))
        #indices_G2=list(range(G1.order(),G1.order()+G2.order()))
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
            sol=list(nx.max_weight_matching(LMG_temp))
            for i,match in enumerate(sol):
                sol[i]=sorted(match)
            Sols.append(sol)
        self._sols=Sols       
    
    

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
               


            
    
        
                
     
#graph_coll=import_graph_coll(path)    
#graph_coll=graph_coll_edit(graph_coll)
G1=graph_coll[12][0]
G2=graph_coll[22][0]
#print(Sols)
E=Evaluator()
ALG=Approx_alg()
GU=Gurobi_solver(0)
#print('new',Sols)

for dist_alg in [ALG,GU]:
    tic=time.time()
    dist_alg.compute_distance(G1,G2)
    tac=time.time()
    print(dist_alg._name,E.evaluate_sol(G1,G2,dist_alg._sol),tac-tic)
    


                


    
#graph_coll=[]
#for dire in lis_dir
