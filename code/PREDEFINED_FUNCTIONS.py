# =============PREDEFINED FUNCTIONS=================

# 01. FUNCTION TO READ GIVEN CSV TO LIST
def read_csv_to_list(filepath):
    import numpy as np

    list = []
    with open(filepath) as openfile:
        for line in openfile.readlines():
            if "#" not in line:
                list.append([float(value) for value in line.replace('\n', '').split(',')])
    return list


# 02. FUNCTION TO READ GIVEN ARRAY TO CSV
def read_array_to_csv(filename, array):
    import csv

    with open(filename, "w", newline='') as writefile:
        csv.writer(writefile).writerows(array)



# 03. FUNCTION TO READ OPTIMAL PATH LIST TO CSV
def read_list_to_csv(filename, list):
    import csv

    with open(filename, "w", newline='') as writefile:
        csv.writer(writefile).writerow(list)



# 04. FUNCTION TO CHECK FOR NODE COLLISION WITH OBSTACLES
def node_collision_check(sample, obstacles):
    '''
    Function to check if there is a collision between sample and any of the obstacles 

    input:
    sample : coordinates of the sample — 1 by 2 array of the form [x, y]
    obstacles : a list of obstacles; each row of the form [x, y, diameter] 

    Returns:
    bool : returns True if collision, else: 
           returns False
    '''
    import numpy as np

    m = np.array(obstacles).shape[0]  # The number of obstacles
    for i in range(m):
        radius = obstacles[i][-1] / 2
        coord = obstacles[i][0:2]

        # computes the Euclidean distance 
        dist = np.linalg.norm(sample - coord)
        if dist <= radius:  # If collision
            a = 1
            break
        else:
            a = 0

    return bool(a)


# 05. FUNCTION TO GENERATE NEW SAMPLE NODES
def node_sampling(N, obstacles):
    '''
    Function to generate N-2 new sample nodes not in collision with obstacles

    input:
    obstacles : a list of obstacles; each row of the form [x,y,diameter] 
    N : total number of nodes to be created in the nodes list

    Returns:
    nodes : a list of N nodes with cost; each of the form [ID,x,y,heuristic-cost-to-go]
    '''

    import numpy as np

    nodes = []
    start = np.array([-0.5, -0.5])
    goal = np.array([0.5, 0.5])
    start_goal= [list(start), list(goal)]

    # compute the start and goal nodes heuristic_ctg to 4 d.p
    start_ctg = np.linalg.norm(start - goal).round(4)
    goal_ctg = np.linalg.norm(goal - goal).round(4)  #This is definitely zero, just computing

    # Append start node details
    nodes.append([1, start[0], start[1], start_ctg])

    i = 1
    while np.array(nodes).shape[0] < N-1:
        # Create a sample coordinate
        a = np.round(np.arange(-0.5, 0.6, 0.1), 1)   # Array of possible sample coordinates
        sample = np.random.choice(a, 2)

        # To make sure the sample is not same as start or goal nodes
        if list(sample) not in start_goal: 
            # check for collision
            collision = node_collision_check(sample, obstacles)
            if collision:
                pass
            else:
                heuristic_ctg = np.linalg.norm(sample - goal).round(4)
                if heuristic_ctg == 0:
                    pass
                else:
                    i += 1
                    new_node =[i,sample[0], sample[1], heuristic_ctg]
                    nodes.append(new_node)   # Append new node details
                    
    # Append goal node details
    nodes.append([N, goal[0], goal[1], goal_ctg]) 

    return nodes


# 06. FUNCTION TO CHECK FOR EDGE COLLISION WITH OBSTACLES
def edge_collision_check(IDs, nodes, obstacles):
    '''
    Function to check if a line segment from ID1 to ID2 in collision with obstacles

    input:
    IDs : an array of [ID1, ID2]
    nodes : a list of N nodes with cost; each of the form [ID,x,y,heuristic-cost-to-go]
    obstacles : a list of obstacles; each row of the form [x,y,diameter] 
    
    Returns:
    bool : returns True if collision, else: 
           returns False
    '''
    import numpy as np
    import math

    ID1, ID2 = IDs
    x_1, y_1 = nodes[ID1-1][1:3]
    x_2, y_2 = nodes[ID2-1][1:3]

    # Getting the parameters of the line segment's equation (ax+bx+c=0)
    a = y_2 - y_1
    b = x_1 - x_2
    c = y_1*(-b) + x_1*(-a)
    
    m = np.array(obstacles).shape[0]  # The number of obstacles
    for i in range(m):
        radius = obstacles[i][-1] / 2
        x_3, y_3 = obstacles[i][0:2]
        # Compute the perpendicular distance between the circle and line
        dist = abs(a*x_3 + b*y_3 + c) / math.sqrt(a*a + b*b)
        
        if (dist == 0) or (dist <= radius+0.006):  # If collision
            z = 1
            break
        else:
            z = 0

    return bool(z)


# 07. FUNCTION TO CREATE EDGES
def get_edges(nodes, k, obstacles):
    '''
    Function to compute the cost of the edges between nodes

    input:
    nodes : a list of N nodes with cost; each of the form [ID,x,y,heuristic-cost-to-go]
    obstacles : a list of obstacles; each row of the form [x,y,diameter]
    k : Number of edges to be generated in the edges list

    Returns:
    edges : a list of edges; each of the form [ID1,ID2,cost]
    '''
    
    import numpy as np

    edges = []
    nodes = np.array(nodes)
    node_IDs = nodes[:,0]
    IDs_list = []

    i = 0
    while i < k:
        IDs = np.random.choice(node_IDs, 2).astype(int)
        ID1, ID2 = IDs

        # To make sure the IDs are not equal or sampled before
        if (list(IDs) not in IDs_list) and (ID1 != ID2):
            IDs_list.append(list(IDs))
            
            # Check if segment is in collision with obstacles
            collision = edge_collision_check(IDs, nodes, obstacles)
            if collision:
                pass
            else:
                ID1_coord = np.array(nodes[ID1-1][1:3])
                ID2_coord = np.array(nodes[ID2-1][1:3])
                
                cost = np.linalg.norm(ID1_coord - ID2_coord).round(4)
                if cost != 0:
                    new_edge =[ID1, ID2, cost]
                    edges.append(new_edge)   # Append new edge details
                    i += 1

    return edges


