import numpy as np
import Queue

def find_valid_points(graph,threshold = 5, max_dist=25):
    explored_graph = np.mean(graph, axis = 2)
    path_graph = graph[:,:,0] #get the blue points aka the path the robot took
    valid_graph = np.mean(graph[:,:,0:2], axis = 2) #get the green points aka where the robot has validated as free
    obstacle_graph = graph[:,:,2] # get the red points aka where is a verified obstacle
    path_distance_graph = np.zeros_like(path_graph)
    obstacle_distance_graph = np.zeros_like(path_graph, dtype=np.float32)
    #------------------------------
    x_all, y_all = np.where(explored_graph)#get list of all indices
    x_path, y_path = np.where(path_graph) #get list of indices of robot path
    x_obstacle, y_obstacle = np.where(obstacle_graph) #get list of indices of obstacles
    min_point = np.min(x_all), np.min(y_all) #find the local box we want the graph to traverse
    max_point = np.max(x_all), np.max(y_all)
    #------------------------------
    traverse_graph(zip(x_obstacle, y_obstacle), obstacle_distance_graph, min_point, max_point)#find distances for each point from closest object
    x_close, y_close = np.where(obstacle_distance_graph < threshold)
    valid_graph[x_close,y_close] = 0 #Find explored points not near an object
    #-----------------------------
    traverse_graph(zip(x_path, y_path), path_distance_graph, min_point, max_point, stop = valid_graph)#find longest distances
    path_distance_graph = np.where(path_distance_graph > max_dist, np.zeros_like(path_distance_graph), path_distance_graph)
    x_valid, y_valid = np.where(valid_graph)
    #----------------------------
    final_graph = np.zeros_like(path_distance_graph)
    final_graph[x_valid, y_valid] =path_distance_graph[x_valid, y_valid] #return map of valid distances
    final_graph[x_path,y_path] = 1
    return final_graph

def traverse_graph(starting_points,container,min_point,max_point, stop=None, diag = True):
    my_queue = Queue.Queue()

    for point in starting_points:
        my_queue.put(point)
    visited_points = set(starting_points)

    while not my_queue.empty():
        point = my_queue.get()
        adj_neighbors  = get_adjacent_neighbors(point, min_point, max_point)
        diag_neighbors  = get_diagonal_neighbors(point, min_point, max_point)
        for n in adj_neighbors:
            if (n not in visited_points):
                visited_points.add(n)
                if (stop is not None) and (not stop[n]):
                    continue
                container[n] = container[point] + 1
                my_queue.put(n)
        if diag:
            for n in diag_neighbors:
                if (n not in visited_points):
                    visited_points.add(n)
                    if (stop is not None) and (not stop[n]):
                        continue
                    container[n] = container[point] + 1.414
                    my_queue.put(n)


def get_adjacent_neighbors(point, min_point, max_point):
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
