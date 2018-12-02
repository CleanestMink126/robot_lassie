
import numpy as np
import matplotlib.pyplot as plt
import graphTraversal

rad2degrees = 180/3.14159
degrees2rad = 3.14159/180

class Map(object):
    def __init__(self, odom, res= .15,size = 400):
        self.offset = odom[0], odom[1]
        self.res = res
        # self.turn_offset = odom.yaw
        self.graph = np.full((size,size,3),0,np.uint8)
        self.size = size
        self.center = size//2, size//2
        self.positions = [[size//2, size//2]]


    def update_graph(self, odom, laser_scan):
        '''
        odom --> x,y,yaw
        laser_scan --> 360 of laser scan data
        '''
        laser_scan = np.array(laser_scan)
        x_val, y_val = self.normalize_laser_scan(odom, laser_scan)
        origin = odom[0] - self.offset[0], odom[1] - self.offset[1]
        green_points = self.find_valid_pixels(origin, x_val,y_val)
        x_index = x_val/self.res# + self.center[0]
        y_index = y_val/self.res# + self.center[1]
        # print(x_index, y_index)
        x_index += self.center[0]
        y_index += self.center[1]
        red_points = np.concatenate([np.expand_dims(x_index,1),np.expand_dims(y_index,1)],1)
        robot_posx = int((odom[0] - self.offset[0])//self.res + self.center[0])
        robot_posy = int((odom[1] - self.offset[1])//self.res + self.center[1])

        self.positions = np.concatenate([self.positions,[[robot_posx,robot_posy]]],axis = 0)
        # print(points)
        if not len(red_points):
            print('Empty Scan')
            return
        if np.min(red_points) < 0 or np.max(red_points) >= self.size:
            print("OUT OF DIMENSIONS (Somehow?)")
            return
        red_points = red_points.astype(np.int32)
        green_points = green_points.astype(np.int32)
        self.graph[green_points[:,0],green_points[:,1],:] = [255,255,255]
        self.graph[red_points[:,0],red_points[:,1],:] = [0,0,204]
        self.graph[self.positions[:,0],self.positions[:,1],:] = [204,0,0]
        distance_graph = graphTraversal.find_valid_points(self.graph, 5)
        index_flat = np.argmax(distance_graph)
        x_goal = index_flat // self.size
        y_goal = index_flat % self.size
        self.goal[x_goal-2:x_goal+2, y_goal-2:y_goal+2,:] = [0,204,0]
        # print(np.max(self.graph))
        # print(self.graph.dtype)
        # print(self.graph)

    def find_valid_pixels(self,origin,x_val, y_val):
        x_val = [np.linspace(origin[0], val,5,False) for val in x_val]
        y_val = [np.linspace(origin[1], val,5,False) for val in y_val]
        x_val = np.reshape(x_val,[-1])
        y_val = np.reshape(y_val,[-1])
        x_index = x_val/self.res# + self.center[0]
        y_index = y_val/self.res# + self.center[1]
        # print(x_index, y_index)
        x_index += self.center[0]
        y_index += self.center[1]
        return np.concatenate([np.expand_dims(x_index,1),np.expand_dims(y_index,1)],1)


    def normalize_laser_scan(self, odom, list_ranges):
        #First get the valid values
        mask = np.where(list_ranges > .75)
        indices = np.arange(len(list_ranges))[mask]
        # print(indices)
        #Get the x, y coordinates compared to Neato (adjusting for Neato angle)
        # print(odom[2])
        x_angled = np.cos(indices.astype(np.float32) * degrees2rad + odom[2]) * list_ranges[indices]
        y_angled = np.sin(indices.astype(np.float32) * degrees2rad + odom[2]) * list_ranges[indices]
        # if np.random.rand() < 0.1:
        #     plt.plot(x_angled, y_angled)
        #     plt.show()
        #Transform x, y coordinates to map frame
        x_transform = odom[0] - self.offset[0]
        y_transform = odom[1] - self.offset[1]
        # print(x_transform)
        #Get final analog coordinates
        x_val = x_angled + x_transform
        y_val = y_angled + y_transform

        return x_val,y_val
