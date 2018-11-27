
import numpy as np

class Map(object):
    def __init__(odom, grains= .1,size = 1000):
        self.offset_x = odom.x
        self.offset_y = odom.y
        self.turn_offset = odom.yaw
        self.graph = np.full((size,size),0)


    def update_graph(self, odom, laser_scan):
        '''
        odom --> x,y,yaw
        laser_scan --> 360 of laer scan data
        '''
        
