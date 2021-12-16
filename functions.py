import pandas as pd
import matplotlib.pyplot as plt
import networkx as nx
import datetime, time
import math
import numpy as np
from tqdm.auto import tqdm

####################EXERCISE 1#########################
def findTimeMaxMin(data,max,min): #Retrieve maximum and minimum timestamps
    m = data['t'].min()
    M = data['t'].max()
    if m < min:
        min = m 
    if M > max:
        max = M
    return max, min

#I create my graph so that for each relationship between two nodes I increase the weight by 1 and I append the moment in which this relationship occurred. 
def rowLevel(G,line):
    t= line['t']
    if G.has_edge(line['u'], line['v']) :
        G[line['u']][line['v']]['weight'] += 1
        G[line['u']][line['v']]['timestamp'].append(line['t'])
    else :
        G.add_edge(line['u'], line['v'], weight=1)
        G[line['u']][line['v']]['timestamp']=[line['t']]
    
def createWeightedGraph(G, data, t_start, t_end):
    Filtered = data[(data['t'] >= t_start) & (data['t'] < t_end)]
    Filtered.apply(lambda x : rowLevel(G,x) , axis =1)
    
    

####################EXERCISE 2#########################
## I add this function which returns the result faster
def isDirected(Graph):
    for u,v,attr in (Graph.edges(data=True)):
        
        if list(Graph.in_edges(u)) != list(Graph.out_edges(u)):
            break
    return True