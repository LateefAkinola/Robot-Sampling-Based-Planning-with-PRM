# =============PRM PLANNER CODE======================
def PRM_Planner(obstacles_filepath, N, k, min_path_length):
    '''
    Function to use a Probabilistic Road Map sampling method for motion planning

    input:
    obstacles_filepath : the path to the 'obstacles.csv' file provided
    N : total number of nodes to be created in the nodes list
    k : Number of edges to be generated in the edges list
    min_path_length : minimum size of path list

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

	# Search with A_Star Algorithm
        path = A_star_search(nodes, edges)

    return path, nodes, edges
        
