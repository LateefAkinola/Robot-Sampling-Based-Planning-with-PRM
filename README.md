# Robot-Sampling-Based-Planning-with-PRM

### GOAL:
To write a PRM program for the implementation of a sampling-based planning, to find a path for a point robot through the cluttered planar environment with predefined obstacles.
The program will choose its own random samples in the (x, y) C-space, which is the square [-0.5, 0.5] x [-0.5, 0.5]. A straight-line planner will be employed and the distance between any two configurations is simply the Euclidean distance
The start configuration is at (x, y) = (-0.5, -0.5), the bottom left corner of the square environment, and the goal configuration is at (x, y) = (0.5, 0.5), the top right corner of the square environment.

The program will take as input the obstacles.csv file and the output of the program will be three files: nodes.csv, edges.csv, and path.csv.

The csv motion planning kilobot scene will be utilized for visualization

#### The CSV Motion Planning Kilobot Scene:
This scene allows you to visualize motion planning on an undirected graph using graph-search techniques such as A*. To visualize the planned motion, we are using the [kilobot](https://www.kilobotics.com/) robot moving in a planar square environment of dimensions -0.5 <= x <= 0.5 and -0.5 <= y <= 0.5. Obstacles are represented as cylinders, and the graph itself is illustrated as blue nodes with yellow edges. The path that the kilobot actually follows is indicated by green edges, and the goal node is in red. See the image to the right. This scene does not do motion planning. Instead, it displays the output of your motion planner. It expects you to provide the path to a folder with four files, named nodes.csv, edges.csv, path.csv, and obstacles.csv:

#### =================IMPORTANT INFOMATION ABOUT THE FILES====================
1. `nodes.csv`: If the graph has N nodes, then this file has N rows. Each row is of the form ID,x,y,heuristic-cost-to-go. ID is the unique integer ID number of the node, and these ID numbers should take values 1 through N. x, y are the (x,y) coordinates of the node in the plane. heuristic-cost-to-go is an optimistic approximation of the shortest path from this node to the goal node (e.g., the Euclidean distance to the goal node). This heuristic information is useful for A-star search but is not represented in the visualization of the path.
2. `edges.csv`: If the graph has E edges, then this file has E rows. Each row is of the form ID1,ID2,cost. ID1 and ID2 are the node IDs of the nodes connected by the edge. cost is the cost of traversing that edge. This file can be empty if you do not wish to display edges.
3. `path.csv`: This file specifies the solution path in the graph, and it is a single line, of the form ID1,ID2,... The first number is the ID of the first node in the solution path, and the last number is the ID of the last node in the solution path. If there is no solution to the motion planning problem, the path can consist of a single ID number, the ID of the node where the robot starts (and stays).
4. `obstacles.csv`: This file specifies the locations and diameters of the circular obstacles. Each row is x, y, diameter, where (x,y) is the center of the obstacle and diameter is the diameter of the obstacle. This file can be empty if there are no obstacles

<> The 'Code' folder consists of three files:

1. `PREDEFINED_FUNCTIONS.py`: This contains 2 predefined functions called in the A-Star fn and during implementation 

2. `A_STAR_SEARCH_FUNCTION_CODE.py`: This computes the function that implements the A-Star Algorithm    

3. `A_STAR_GRAPH_CODE.ipynb`: This notebook contains all the functions and their implementation.

<> The 'CSV' folder consists of all the `csv files`

<> `CoppeliaSim_Screenshot.png`: A CoppeliaSim screenshot of the solution path on the graph

#### NOTE:
The edges 4 to 7 was excluded from the edges file. That was done by commenting it out in the `edges.csv`.