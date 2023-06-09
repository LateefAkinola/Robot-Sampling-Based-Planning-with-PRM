# ================IMPLEMENTING THE CODE================

# CALLING THE FUNCTIONS

obstacles_filepath = 'results\obstacles.csv'
N = 15  # Total Number of Nodes to be Sampled
k = 15  # Total Number of Edges to be Created
min_path_length = 3

path, nodes, edges = PRM_Planner(obstacles_filepath, N, k, min_path_length)
read_list_to_csv("results\path.csv", path)
read_array_to_csv("results\\nodes.csv", np.array(nodes))
read_array_to_csv("results\edges.csv", np.array(edges))

# --------------------------------------
# OUTPUT:

# The Optimal Path is: [ 1.  8. 14. 15.]

#Also, it would have saved these files:
# path as 'path.csv', nodes as 'nodes.csv', and edges as 'edges.csv'