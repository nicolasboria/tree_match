#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 18:20:28 2021

@author: nboria
"""

def read_tree(path):
    with open(path) as fp:
        tree_string = fp.readline().strip()
    return tree_string

def parse_tree(nodes, edges, tree_string, num_nodes):
    node_id = num_nodes
    nodes.append({'id': node_id, 'label': tree_string[1]})
    num_nodes += 1
    tree_string = tree_string[2:-1]
    start = 0
    num_open = 0
    for i in range(len(tree_string)):
        if tree_string[i] == '{':
            num_open += 1
        elif tree_string[i] == '}':
            num_open -= 1
        if num_open == 0:
            edges.append((node_id, num_nodes))
            num_nodes = parse_tree(nodes, edges, tree_string[start:i+1], num_nodes)
            start = i+1
    return num_nodes

def bracket_to_gml(input_path, output_path):
    tree_string = read_tree(input_path)
    nodes = []
    edges = []
    _ = parse_tree(nodes, edges, tree_string, 0)
    with open(output_path, 'w') as fp:
        fp.write('graph [\n directed 1 \n')
        for node in nodes:
            fp.write(f'  node [\n    id {node["id"]}\n    lbl "{node["label"]}"\n  ]\n')
        for edge in edges:
            fp.write(f'  edge [\n    source {edge[0]}\n    target {edge[1]}\n  ]\n')
        fp.write(']\n')
