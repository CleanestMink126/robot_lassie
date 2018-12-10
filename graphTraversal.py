import numpy as np
import Queue

def find_valid_points(graph,threshold):
    explored_graph = np.mean(graph, axis = 2)
    path_graph = graph[:,:,0]
    valid_graph = graph[:,:,1]
    obstacle_graph = graph[:,:,2]
    path_distance_graph = np.zeros_like(path_graph)
    obstacle_distance_graph = np.zeros_like(path_graph)

    x_all, y_all = np.where(explored_graph)
    x_path, y_path = np.where(path_graph)
    x_valid, y_valid = np.where(valid_graph)
    x_obstacle, y_obstacle = np.where(obstacle_graph)
    # print(x_all)
    min_point = np.min(x_all), np.min(y_all)
    max_point = np.max(x_all), np.max(y_all)
    # print(min_point,max_point)

    traverse_graph(zip(x_path, y_path), path_distance_graph, min_point, max_point)
    traverse_graph(zip(x_obstacle, y_obstacle), obstacle_distance_graph, min_point, max_point)
    print(np.max(path_distance_graph))
    x_close, y_close = np.where(obstacle_distance_graph < threshold)
    final_graph = np.zeros_like(path_distance_graph)
    final_graph[x_valid, y_valid] =path_distance_graph[x_valid, y_valid]
    final_graph[x_close,y_close] = 0
    return final_graph

def traverse_graph(starting_points,container,min_point,max_point):
    my_queue = Queue.Queue()

    for point in starting_points:
        my_queue.put(point)
    visited_points = set(starting_points)

    while not my_queue.empty():
        point = my_queue.get()
        neighbors  = get_neighbors(point, min_point, max_point)
        for n in neighbors:
            if n not in visited_points:
                my_queue.put(n)
                container[n] = container[point] + 1
                visited_points.add(n)

def get_neighbors(point, min_point, max_point):
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
