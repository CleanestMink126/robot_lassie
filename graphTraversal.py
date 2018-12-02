import numpy as np
import queue

def find_valid_points(graph, path):
    explored_graph = np.mean(graph[:,:,:2], axis = 2)
    distance_graph = np.zeros_like(graph[:,:,0])
    my_queue = queue.SimpleQueue()
    x_all, y_all = np.where(explored_graph)
    all_points = zip(x_all, y_all)
    all_points = set(all_points)
    visited_points = zip(path[:,0], path[:,1])
    for point in visited_points:
        my_queue.put(point)
    visited_points = set(visited_points)
    while not my_queue.empty():
        point = my_queue.get()



    print(all_points)
    print((1,2) in all_points)
    # for val in/

def get_neighbors(point, explored_graph, max_index):


if __name__ == "__main__":
    graph = np.ones((5,5,3))
    graph[1,2] = [10,20,30]
    graph[1,4] = [10,20,30]
    graph[3,1] = [10,20,30]
    path = np.array([[0,0],[1,1]])
    find_valid_points(graph,path)
