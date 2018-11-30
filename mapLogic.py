
import numpy as np

class Map(object):
    def __init__(odom, res= .1,size = 300):
        self.offset = odom[0], odom[1]
        self.res = res
        # self.turn_offset = odom.yaw
        self.graph = np.full((size,size),0)
        self.size = size
        self.center = size//2, size//2


    def update_graph(self, odom, laser_scan):
        '''
        odom --> x,y,yaw
        laser_scan --> 360 of laser scan data
        '''
        laser_scan = np.array(laser_scan)
        x_val, y_val = self.normalize_laser_scan(odom, laser_scan)
        x_index = x_val//self.res + self.center[0]
        y_index = y_val//self.res + self.center[1]
        points = np.concatenate([np.expand_dims(x_index),np.expand_dims(y_index)],1)
        if np.min(points) < 0 or np.max(points) >= self.size:
            print("OUT OF DIMENSIONS (Somehow?)")
            return

        self.graph[point[0],point[1]] =1


    def normalize_laser_scan(self, odom, list_ranges):
        #First get the valid values
        mask = np.where(list_ranges)
        indices = np.arange(len(list_ranges))[mask]
        #Get the x, y coordinates compared to Neato (adjusting for Neato angle)
        x_angled = np.cos(indices * degrees2rad + odom[2]) * list_ranges[indices]
        y_angled = np.sin(indices * degrees2rad + odom[2]) * list_ranges[indices]
        #Transform x, y coordinates to map frame
        x_transform = odom[0] - self.offset[0]
        y_transform = odom[1] - self.offset[1]
        #Get final analog coordinates
        x_val = x_angled + x_transform
        y_val = y_angled + y_transform

        return x_val,y_val
