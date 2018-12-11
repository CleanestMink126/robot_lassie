import numpy as np
import Queue

def find_valid_points(graph,threshold = 5, max_dist=25):
    '''This is the main graph traversal function. It will find all explored points that
    are a certain distance from an obstacle (so the neato won't get caught)
    and will tell their distance from the closest path the neato has traversed
    graph  ---> BGR map
    threshold-> how far away points need to be from an obstacle
    max_dist--> maximum distance points can be from the traveled path
    '''
    #-------------------------------
    #Segment the input graph
    explored_graph = np.mean(graph, axis = 2)#all seen points
    path_graph = graph[:,:,0] #get the blue points aka the path the robot took
    valid_graph = np.mean(graph[:,:,0:2], axis = 2) #get the green points aka where the robot has validated as free
    obstacle_graph = graph[:,:,2] # get the red points aka where is a verified obstacle
    path_distance_graph = np.zeros_like(path_graph)
    obstacle_distance_graph = np.zeros_like(path_graph, dtype=np.float32)
    #------------------------------
    #Get the relevent points as indices
    x_all, y_all = np.where(explored_graph)#get list of all indices
    x_path, y_path = np.where(path_graph) #get list of indices of robot path
    x_obstacle, y_obstacle = np.where(obstacle_graph) #get list of indices of obstacles
    min_point = np.min(x_all), np.min(y_all) #find the local box we want the graph to traverse
    max_point = np.max(x_all), np.max(y_all)
    #------------------------------
    ##Find all points close to an obstacle
    traverse_graph(zip(x_obstacle, y_obstacle), obstacle_distance_graph, min_point, max_point)#find distances for each point from closest object
    x_close, y_close = np.where(obstacle_distance_graph < threshold)
    valid_graph[x_close,y_close] = 0 #Remove close points form validated graph
    #-----------------------------
    #Find the distances of all valid points
    traverse_graph(zip(x_path, y_path), path_distance_graph, min_point, max_point, stop = valid_graph)#find longest distances
    path_distance_graph = np.where(path_distance_graph > max_dist, np.zeros_like(path_distance_graph), path_distance_graph)#Remove far away points
    x_valid, y_valid = np.where(valid_graph)#Get relevent indices
    #----------------------------
    final_graph = np.zeros_like(path_distance_graph) #define a final mask of valid points
    final_graph[x_valid, y_valid] =path_distance_graph[x_valid, y_valid] #return map of valid distances
    final_graph[x_path,y_path] = 1 #Reset traveled path to one (so it doesn't think the area is invalid)
    return final_graph

def traverse_graph(starting_points,container,min_point,max_point, stop=None):
    '''
    This will preform a BFS from some given starting points, denoting distance from
    the starting points and exploring according to min, max, and an optional
    stop matrix
    starting_points --> initial point from which to start exploring
    container       --> placeholder to store distance from starting points
    min/max_point   --> region of interest in the graph, won't explore outside this region
    stop            --> matrix where zero values indicate to not explore those pixels
    '''
    my_queue = Queue.Queue() #main queue for BFS
    for point in starting_points:
        my_queue.put(point)

    visited_points = set(starting_points)#define visited vertices

    while not my_queue.empty():
        point = my_queue.get()
        adj_neighbors  = get_adjacent_neighbors(point, min_point, max_point)
        diag_neighbors  = get_diagonal_neighbors(point, min_point, max_point)

        for n in adj_neighbors:
            if (n not in visited_points):
                visited_points.add(n)
                if (stop is not None) and (not stop[n]):
                    continue
                container[n] = container[point] + 1 #distance for adjacent
                my_queue.put(n)

        for n in diag_neighbors:
            if (n not in visited_points):
                visited_points.add(n)
                if (stop is not None) and (not stop[n]):
                    continue
                container[n] = container[point] + 1.414 #root(2)
                my_queue.put(n)


def get_adjacent_neighbors(point, min_point, max_point):
    '''Return adjacent neighbors in correct bounds for a given 2D point'''
    neighbors = []
    if point[0] -1 >= min_point[0]:
        neighbors.append((point[0]-1,point[1]))
    if point[1] -1 >= min_point[1]:
        neighbors.append((point[0],point[1]-1))
    if point[0] +1 < max_point[0]:
        neighbors.append((point[0]+1,point[1]))
    if point[1] +1 < max_point[1]:
        neighbors.append((point[0],point[1]+1))
    return neighbors

def get_diagonal_neighbors(point, min_point, max_point):
    '''Return diagonal neighbors in correct bounds for a given 2D point'''
    neighbors = []
    x_min = point[0] -1 >= min_point[0]
    y_min = point[1] -1 >= min_point[1]
    x_max = point[0] +1 < max_point[0]
    y_max = point[1] +1 < max_point[1]
    if x_min and y_min:
        neighbors.append((point[0]-1,point[1]-1))
    if x_min and y_max:
        neighbors.append((point[0]-1,point[1]+1))
    if x_max and y_min:
        neighbors.append((point[0]+1,point[1]-1))
    if x_max and y_max:
        neighbors.append((point[0]+1,point[1]+1))
    return neighbors


if __name__ == "__main__":
    graph = np.array(
    [
    [[1,0,0,0,0],
    [0,1,0,0,0],
    [0,0,0,0,0],
    [0,0,0,0,0],
    [0,0,0,0,0]],
    #---------
    [[0,1,1,1,0],
    [1,0,1,1,0],
    [1,1,1,1,0],
    [1,1,1,1,0],
    [1,1,1,1,0]],
    #--------
    [[0,0,0,0,1],
    [0,0,0,0,1],
    [0,0,0,0,1],
    [0,0,0,0,1],
    [0,0,0,0,1]]
    ]
    )
    graph = np.transpose(graph,[1,2,0])
    distance_graph = find_valid_points(graph, 2)
    print(distance_graph)
