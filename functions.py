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

# def delete_edges(G,time_start,end_time): #remove edges which are not in the time lapse
#     edge_to_drop=[]
    
#     for u,v,att in G.edges(data=True):
#         if not time_start <= att['timestamp'] <= end_time:
#             edge_to_drop.append((u, v))
#     [G.remove_edge(u,v) for (u,v) in set(edge_to_drop)]
#     return G
def timestamp_conversion(date):
    date = int(time.mktime(datetime.datetime.strptime(date, "%Y/%m/%d").timetuple()))
    return date

def delete_edges(G,time_start,end_time):
    start = timestamp_conversion(time_start)
    end = timestamp_conversion(end_time)
    edges = [(u,v,d) for u,v,d in G.edges(data = True) if (start < d['timestamp'][0] < end)]
    H = nx.DiGraph()
    H.add_edges_from(edges)
    return H

        
def defineTime(myTime):#return the time in seconds
    myTime = time.mktime(myTime.timetuple())
    return myTime

#-------------------------------------------------------------------------------------------------------------------
def Dijkstra_SP(Graph, source, target):
    
    dist = {node : float('infinity') for node in list(nx.nodes(Graph))} # dict of the distances from each node to source
    previous = {node : None for node in list(nx.nodes(Graph))} # previous node
    
    unvisited = list(nx.nodes(Graph)) # all nodes
    
    dist[source] = 0 # The distance of the source node is 0
    
    while unvisited :  # until we have visited all nodes
        
        # visit the unvisited node with the smallest known distance
        dist_min = float('infinity')
        for node in unvisited:
            if dist[node] <= dist_min :
                dist_min = dist[node]
                current = node
        unvisited.remove(current) # removing the node from the list of the unvisited nodes
        
        #If the visited node is the target stop the visits
        if current == target:
            break
            
        # Examine its neighbours
        for neighbor in Graph.neighbors(current):
            # calulate the distance of each neighbour from the node
            new_dist = dist[current] + Graph[current][neighbor]['weight']
            # if the new distance is lower than the previous one we update di distances dict
            if dist[neighbor] > new_dist:
                # update the shortest distance
                dist[neighbor] = new_dist
                # update the previous node
                previous[neighbor] = current
                
    # if the target hasn't a previous node it means that the graph is not connected 
    if previous[target] == None:
        return ('Not Connected',[])
    
    path = [target]
    while source not in path:
        path.append(previous[path[-1]])
    path.reverse()

    return dist[target],path # return the distance

#find a user which isn't in both graph
def findSpecialUser(G,H):
    for node in G.nodes():
        if not H.has_node(node):
            return node
    raise ValueError('A very specific bad thing happened.')




#----------------------test code for betweeness

def allShortestPaths(G,start, goal):
    distance = 'start'
    minDistance = None
    all_path = []
    
    while distance!='Not Connected':
        distance, path = Dijkstra_SP(G,start, goal);
        if minDistance == None:
            minDistance = distance 
        
        if(len(path)>0):
            for x in path:
                if x != start and x!=goal:
                    G.remove_node(x)
            if distance == minDistance:
                all_path.append(path)
        
            if(path[0] == start and path[1] == goal):
                return 1
    return len(all_path)

def allShortestPathsWithV(G,start, goal, v):
    distance = 'start'
    minDistance = None
    all_path = []
    
    if start == goal:
        return 0
    
    while distance!='Not Connected':
        distance, path = Dijkstra_SP(G,start, goal);
        if minDistance == None:
            minDistance = distance 
        
        if(len(path) == 1):
            return 0
        if(len(path)>0):
            for x in path:
                if x != start and x!=goal and x!=v:
                    G.remove_node(x)
            if distance == minDistance:
                all_path.append(path)
            if(path[0] == start and path[1] == goal):
                return 1
            
        
    return len(all_path)

### visualization 2
# Python3 code to demonstrate working of
# Convert date range to N equal durations
# Using loop
def equi_intervals(start_date, end_date, N):
    # initializing dates
    test_date1 = datetime.datetime.strptime(start_date, '%Y/%m/%d')
    test_date2 = datetime.datetime.strptime(end_date, '%Y/%m/%d')

    N = N
    temp = []

    # getting diff.
    diff = ( test_date2 - test_date1) // N
    for idx in range(0, N):

        # computing new dates
        temp.append((test_date1 + idx * diff))

    res = []
    for sub in temp:
        res.append(sub.strftime("%Y/%m/%d"))

    # printing result
    return res + [end_date]