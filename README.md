# Robot-Sampling-Based-Planning-with-PRM

### GOAL:
To write a PRM program for the implementation of a sampling-based planning, to find a path for a point robot through the cluttered planar environment with predefined obstacles.
The program will choose its own random samples in the (x, y) C-space, which is the square [-0.5, 0.5] x [-0.5, 0.5]. A straight-line planner will be employed and the distance between any two configurations is simply the Euclidean distance
The start configuration is at (x, y) = (-0.5, -0.5), the bottom left corner of the square environment, and the goal configuration is at (x, y) = (0.5, 0.5), the top right corner of the square environment.

The program will take as input the obstacles.csv file and the output of the program will be three files: nodes.csv, edges.csv, and path.csv.

_The csv motion planning kilobot scene will be utilized for visualization_

#### The CSV Motion Planning Kilobot Scene:
This scene allows you to visualize motion planning on an undirected graph using graph-search techniques such as A*. To visualize the planned motion, we are using the [kilobot](https://www.kilobotics.com/) robot moving in a planar square environment of dimensions -0.5 <= x <= 0.5 and -0.5 <= y <= 0.5. Obstacles are represented as cylinders, and the graph itself is illustrated as blue nodes with yellow edges. The path that the kilobot actually follows is indicated by green edges, and the goal node is in red. See the image to the right. This scene does not do motion planning. Instead, it displays the output of your motion planner. It expects you to provide the path to a folder with four files, named nodes.csv, edges.csv, path.csv, and obstacles.csv:

#### =================IMPORTANT INFOMATION ABOUT THE FILES====================
1. `nodes.csv`: If the graph has N nodes, then this file has N rows. Each row is of the form ID,x,y,heuristic-cost-to-go. ID is the unique integer ID number of the node, and these ID numbers should take values 1 through N. x, y are the (x,y) coordinates of the node in the plane. heuristic-cost-to-go is an optimistic approximation of the shortest path from this node to the goal node (e.g., the Euclidean distance to the goal node). This heuristic information is useful for A-star search but is not represented in the visualization of the path.
2. `edges.csv`: If the graph has E edges, then this file has E rows. Each row is of the form ID1,ID2,cost. ID1 and ID2 are the node IDs of the nodes connected by the edge. cost is the cost of traversing that edge. This file can be empty if you do not wish to display edges.
3. `path.csv`: This file specifies the solution path in the graph, and it is a single line, of the form ID1,ID2,... The first number is the ID of the first node in the solution path, and the last number is the ID of the last node in the solution path. If there is no solution to the motion planning problem, the path can consist of a single ID number, the ID of the node where the robot starts (and stays).
4. `obstacles.csv`: This file specifies the locations and diameters of the circular obstacles. Each row is x, y, diameter, where (x,y) is the center of the obstacle and diameter is the diameter of the obstacle. This file can be empty if there are no obstacles



#### ================================== PRM PLANNING ==================================
To start with, functions to read 'obstacles.csv' to list and to save nodes, edges, and path
to csv was written. 

`Sampling`:=> Two functions was written for implement sampling; one to check if a randomly generated node is not in collision with obstacles, and the other to create a list of nodes confirmed as not in collision.
`Creating Edges`:=> Two functions was written for creating edges; one to check if a randomly created edge is not in collision with obstacles, and the other to create a list of edges confirmed as not in collision.
`Searching`:=> The Roadmap was searched using the A_Star Algorithm code for the optimal path.

##### ===========================================================
`input`: The PRM_PLANNING function takes 4 inputs:
	    => `obstacles_filepath` : the path to the 'obstacles.csv' file provided
    	=> `N` : total number of nodes to be created in the nodes list
    	=> `k`: Number of edges to be generated in the edges list
    	=> `min_path_length` : minimum size of path list

`It has been observed that the best choices for the hyperparameters are`:
	=> N = 10 - 15  
	=> k = 10 - 15  
	=> min_path_length = 3–5

##### ==========================================================
<> *The `'Code' folder` consists of three files*:

1. `PREDEFINED_FUNCTIONS.py`: This contains 7 predefined functions called in the A-Star, PRM functions, and during implementation.

2. `A_STAR_SEARCH_FUNCTION_CODE.py`: This computes the function that implements the A-Star Graph Search Algorithm.

3. `PRM_PLANNER_CODE.py`: This computes the function that implements executes the PRM sampling.

4. `IMPLEMENTATION.py`: This shows the implementation of the fns and the output

<> Each `'results_1' and 'results_2' folder` consists of all the `csv files` and their respective screenshots showing the solution path on the graph

<> The `PRM-Sampling-Based-Planner.ipynb` file: This notebook contains all the functions and their implementation.