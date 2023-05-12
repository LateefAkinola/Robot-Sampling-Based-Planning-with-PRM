# =============A STAR SEARCH CODE======================
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
    print(f"No Optimal Path found yet; The robot goes no where!")
    return path
