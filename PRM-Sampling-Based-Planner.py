# Import libraries
import numpy as np
import csv

# Function
def read_csv_to_list(filepath):
    import numpy as np

    list = []
    with open(filepath) as openfile:
        for line in openfile.readlines():
            if "#" not in line:
                list.append([float(value) for value in line.replace('\n', '').split(',')])
    return list

# Function
def read_list_to_csv(filename, list):
    import csv

    with open(filename, "w", newline='') as writefile:
        csv.writer(writefile).writerow(list)


# Function
def A_star_search(nodes, edges):
    import numpy as np

    '''
    Function to find and print an optimal path using the A* algorithm
    If path is found, it saves as "path.csv" 

    input:
    nodes : a list of nodes; each of the form [ID,x,y,heuristic-cost-to-go]
    edges : a list of edges; each of the form [ID1,ID2,cost] 

    Returns:
    path : a list of the form [ID1,ID2,...] if part is found; else: 
           returns 1
    '''

    # Initializing key variables
    nodes_matrix = np.array(nodes)
    N = nodes_matrix.shape[0]
    OPEN =[]
    CLOSED = []
    past_cost = np.zeros(N) + np.inf
    past_cost[0] = 0.
    est_total_cost = np.zeros(N)
    parent = np.zeros(N)
    current = 0

    # Append Start Node to OPEN
    node = nodes[0][0]
    ctg = nodes[0][-1]
    est_total_cost[0] = past_cost[0] + ctg
    OPEN.append([node, est_total_cost[0]])

    #THE LOOP
    while len(OPEN) != 0:      
        current = int(OPEN[0][0])
        OPEN.pop(0)
        CLOSED.append(current)
          
        if current==(nodes[-1][0]):
            path_hist = []      # Instantiate a list of path history
            current = int(current)
            while current != 1:
                path_hist.append(current)
                current = parent[int(current)-1]
            if current == 1:
                path_hist.append(current)
            path = np.zeros(len(path_hist))
            for i in range(len(path)):
                hist = int(path_hist[len(path)-i-1])
                path[i] = hist
            
            # print optimal path
            print(f"The Optimal Path is: {path}")
            return path
        
        # Make a list of connected nodes not in CLOSED list
        con_list = []
        for edge in edges:
            if edge[0] == current:
                con_list.append(edge[1])
            if edge[1] == current:
                con_list.append(edge[0])
        con_list = list(set(con_list) - set(CLOSED))

        # Calculate the total cost for neighbouring nodes
        for nbr in con_list:
            for edge in edges:
                if (edge[1]==nbr and edge[0]==current) or (edge[1]==current and edge[0] == nbr):
                    nbr = int(nbr)
                    tentative_past_cost = past_cost[int(current)-1] + edge[-1]
                    if tentative_past_cost < past_cost[nbr-1]:
                        past_cost[nbr-1] = tentative_past_cost
                        parent[nbr-1] = current
                        h_ctg = nodes[nbr-1][-1]
                        est_total_cost[nbr-1] = past_cost[nbr-1] + h_ctg
                        OPEN.append([nbr, est_total_cost[nbr-1]])

                        # Sort OPEN list according to the est_total_cost
                        OPEN = sorted(OPEN, key=lambda x:x[-1])
    
    #If no optimal path is found:
    path = [1]
    print(f"No Optimal Path found; The robot goes no where!")
    return path


# Function
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


# Function
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

    
# Function
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

        # Compute the perpendicular distance
        dist = abs(a*x_3 + b*y_3 + c) / np.sqrt(a**2 + b**2)
        # print('dist', dist)

        if (dist == 0) or (dist <= radius):  # If collision
            a = 1
            break
        else:
            a = 0

    return bool(a)


# Function
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
                    # print('edge',new_edge)
                    edges.append(new_edge)   # Append new edge details
                    i += 1

    return edges
    

# Function
def PRM_Planner(obstacles_filepath, N, k, min_path_length):
    '''
    Function to use a Probabilistic Road Map sampling method for motion planning

    input:
    obstacles_filepath : the path to the 'obstacles.csv' file provided
    N : total number of nodes to be created in the nodes list
    k : Number of edges to be generated in the edges list

    Returns:
    path : a list of optimal path; of the form [ID1,ID2,...]
    nodes : a list of N nodes with cost; each of the form [ID,x,y,heuristic-cost-to-go]
    edges : a list of edges; each of the form [ID1,ID2,cost]
    '''
 
    path = [1]
    while len(path) < min_path_length:
        obstacles = read_csv_to_list(obstacles_filepath)
        nodes = node_sampling(N, obstacles)
        edges = get_edges(nodes, k, obstacles)

        path = A_star_search(nodes, edges)

    return path, nodes, edges


# Function
def read_array_to_csv(filename, array):
    import csv

    with open(filename, "w", newline='') as writefile:
        csv.writer(writefile).writerows(array)


# Implement Functions
obstacles_filepath = 'Results\obstacles.csv'  # Import Obstacles
N = 20
k = 40
min_path_length = 2

path, nodes, edges = PRM_Planner(obstacles_filepath, N, k, min_path_length)
read_list_to_csv("Results\path.csv", path)
read_array_to_csv("Results\\nodes.csv", np.array(nodes))
read_array_to_csv("Results\edges.csv", np.array(edges))
