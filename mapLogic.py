
import numpy as np
import matplotlib.pyplot as plt
import graphTraversal

rad2degrees = 180/3.14159
degrees2rad = 3.14159/180

class Map(object):
    def __init__(self, odom, res= .15,size = 400):
        self.offset = odom[0], odom[1] #transformation from Neato coordinates to the map's
        self.res = res #resolution of map
        self.graph = np.full((size,size,3),0,np.uint8) #the main graph
        self.size = size
        self.center = size//2, size//2
        self.positions = [[size//2, size//2]] #past positions of Neato (starts in center)


    def update_graph(self, odom, laser_scan):
        '''
        Given a laser scan and position, update the map with verified clear
        space (green channel) and obstances (red channel)
        odom --> x,y,yaw
        laser_scan --> 360 of laser scan data
        '''
        laser_scan = np.array(laser_scan)
        x_val, y_val = self.normalize_laser_scan(odom, laser_scan) #get coordinates in relation to neato
        x_index = x_val/self.res + self.center[0] #correct for map resoltuion and array center
        y_index = y_val/self.res + self.center[1]
        #-----------------------
        origin = odom[0] - self.offset[0], odom[1] - self.offset[1]
        green_points = self.find_valid_pixels(origin, x_val,y_val) #get clear pixels
        red_points = np.concatenate([np.expand_dims(x_index,1),np.expand_dims(y_index,1)],1) #get obstacles
        #-----------------------
        robot_posx = int((odom[0] - self.offset[0])//self.res + self.center[0]) #get robot position
        robot_posy = int((odom[1] - self.offset[1])//self.res + self.center[1])
        #------------------------
        self.positions = np.concatenate([self.positions,[[robot_posx,robot_posy]]],axis = 0)#update pas poisitions
        if not len(red_points): #Error checking for laser scan
            print('Empty Scan')
            return
        if np.min(red_points) < 0 or np.max(red_points) >= self.size: #if our map isn't big enough
            print("OUT OF DIMENSIONS (Somehow?)")
            return
        #----------------------Reset points on graph
        red_points = red_points.astype(np.int32)
        green_points = green_points.astype(np.int32)
        self.graph[green_points[:,0],green_points[:,1],:] = [0,204,0]
        self.graph[red_points[:,0],red_points[:,1],:] = [0,0,204]
        self.graph[self.positions[:,0],self.positions[:,1],:] = [204,0,0]

    def set_goal(self):
        '''Given a graph, set a likely destination to get more information on
        the environment'''
        #Get the ditance of all valid points
        distance_graph = graphTraversal.find_valid_points(self.graph, 5)
        #Select the furthest away valid point
        index_flat = np.argmax(distance_graph)
        x_goal = index_flat // self.size
        y_goal = index_flat % self.size
        #Use the existing graph information to create a path to the goal that
        #will not collide with any obstacles
        path = self.get_path_to_goal(distance_graph, (x_goal,y_goal)).astype(np.float32)
        path[:,0] = ((path[:,0] - self.center[0])  * self.res) + self.offset[0] #convert to appropriate frame
        path[:,1] = ((path[:,1] - self.center[1])  * self.res) + self.offset[1]
        return path, (x_goal,y_goal),distance_graph

    def get_path_to_goal(self,distance_graph, goal):
        '''Given some valid points and a goal, find a path to the goal.
        We do this by defining the distance from all relevent points to the goal
        then walking from the start to the goal via minimum neighbor distance'''
        current_pos = self.positions[-1] #get starting location
        #-----------------Get bounds for problem
        x_all, y_all = np.where(distance_graph)
        min_point = np.min(x_all), np.min(y_all) #find the local box we want the graph to traverse
        max_point = np.max(x_all)+1, np.max(y_all)+1
        #-----------------Define initial graph
        distance_from_goal = np.full(distance_graph.shape,distance_graph.shape[0] * 2, dtype=np.float32)
        distance_from_goal[goal] = 0 #set origin to 0
        #------------------Define distance to goal for each point
        graphTraversal.traverse_graph([goal], distance_from_goal, min_point, max_point, distance_graph)
        #-----------------Define initial point and a set to make sure we dont run into a loop
        path = np.array([current_pos])
        visited = set()
        while path[-1,0] != goal[0] or path[-1,1] != goal[1]: #not at goal
            all_neighbors = graphTraversal.get_adjacent_neighbors(path[-1],min_point,max_point)\
             + graphTraversal.get_diagonal_neighbors(path[-1],min_point,max_point)
            n_ind = np.array(all_neighbors) #Get all neighbors
            index_min = np.argmin(distance_from_goal[n_ind[:,0],n_ind[:,1]]) #get index of neighbor closest to goal
            next_point = all_neighbors[index_min]
            #-----------
            if next_point in visited: #if we have been here before
                print('Goal',goal) # (shouldn't happen)
                print('Goal val',distance_from_goal[goal])
                print(current_pos)
                plt.imshow(distance_from_goal)
                plt.show()
                exit()
            #----------Add point to list and continue
            visited.add(next_point)
            path = np.concatenate([path, [next_point]], axis = 0)
        return path

    def find_valid_pixels(self,origin,x_val, y_val):
        '''For all the found obstacle, we can assume there is nothing directly
        between us and the obstacle. So we can define a set number of evenly
        spaced points between the two as clear'''
        x_val = [np.linspace(origin[0], val,8,False) for val in x_val]
        y_val = [np.linspace(origin[1], val,8,False) for val in y_val]
        x_val = np.reshape(x_val,[-1])
        y_val = np.reshape(y_val,[-1])
        x_index = x_val/self.res + self.center[0]
        y_index = y_val/self.res + self.center[1]
        return np.concatenate([np.expand_dims(x_index,1),np.expand_dims(y_index,1)],1)


    def normalize_laser_scan(self, odom, list_ranges):
        '''Transform the laser scan from polar to cartesian coordinates'''
        #First get the valid values
        mask = np.where(list_ranges > .25)
        indices = np.arange(len(list_ranges))[mask]
        #Get the x, y coordinates compared to Neato (adjusting for Neato angle)
        x_angled = np.cos(indices.astype(np.float32) * degrees2rad + odom[2]) * list_ranges[indices]
        y_angled = np.sin(indices.astype(np.float32) * degrees2rad + odom[2]) * list_ranges[indices]
        #Transform x, y coordinates to map frame
        x_transform = odom[0] - self.offset[0]
        y_transform = odom[1] - self.offset[1]
        #Get final analog coordinates
        x_val = x_angled + x_transform
        y_val = y_angled + y_transform
        return x_val,y_val
