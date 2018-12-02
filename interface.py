""" Investigate receiving a message using a callback function """
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
##############Messages
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from neato_node.msg import Accel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import message_filters

###############################################################################
#Sending classes
###############################################################################
class SendSpeed(object):
    '''
    Class container that handles sending the speed to a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        rospy.init_node('interface')
        '''Initializer will return instance from which you can call the important
        functions'''
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def send_speed(self,move_forward = None, turn_left = None):
        '''Sends a translational and rotational speed to the neato. Since the
        neato can only control one axis in both translational and rotational
        movement there are only 2 inputs
        move_forward:
            positive --> move forward
            negative --> move backward
        turn_left:
            positive --> turn left
            negative --> turn right'''
        if move_forward == None:
            move_forward = 0
        if turn_left == None:
            turn_left = 0
        my_point_stamped = Twist(linear=Vector3(move_forward,0,0), angular=Vector3(0,0,turn_left))
        self.publisher.publish(my_point_stamped)

    def low_rider(self):
        '''Ignore
        Joke function to make the neato bounce.
        Like so: https://www.youtube.com/watch?v=HEEHmiAR71Y'''
        t = 10
        r = rospy.Rate(5)
        for i in range(t):
            my_point_stamped = Twist(linear=Vector3(1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()
            my_point_stamped = Twist(linear=Vector3(-1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()



class SendMarker(object):
    '''
    Class container that handles sending the markers to a running neato node.
    It should be imported and used as needed by other scripts.
    '''
    def __init__(self):
        rospy.init_node('interface')
        '''Initializer will return instance from which you can call the important
        functions'''
        self.publisher = rospy.Publisher('/my_viz', Marker, queue_size=10)

    def update_marker(self,line_array, frame_id = 'odom'):
        my_line = Marker(action = 0, type= 4, id=0, header=Header(frame_id=frame_id))
        for p in line_array:
            my_line.points.append(Point(x=p[0],y=p[1], z=0) )
        my_line.scale.x = 0.2
        my_line.scale.y = 0.2
        my_line.color.b = 1.0
        my_line.pose.orientation.w = 1.0
        my_line.ns = "voronoi_2D_node"
        my_line.color.a = 1.0
        self.publisher.publish(my_line)

###############################################################################
#Receiving classes
###############################################################################

class BaseLidar(object):
    def __init__(self):
        '''Class currently only supports the range attribute of the Lidar
        message.'''
        rospy.init_node('interface')
        scan_sub = message_filters.Subscriber("/stable_scan", LaserScan)
        odom_sub = message_filters.Subscriber("/odom", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([scan_sub, odom_sub], 10,.05)
        ts.registerCallback(self.process_range)
        rospy.Subscriber("/odom", Odometry,self.process_odom)

        self.get_lidar = False
        self.last_ranges = None
        self.last_odom = None
        self.current_odom = None
        print('BaseLidar')

    def process_range(self, scan_m, odom_m):
        print('Range Callback')
        if self.get_lidar:
            self.last_ranges = scan_m.ranges
            self.last_odom = BaseLidar.convert_odom(odom_m)
            self.get_lidar = False

    def process_odom(self, m):
        self.current_odom = m

    def reset(self):
        self.last_ranges = None
        self.last_odom = None

    @staticmethod
    def convert_odom(odom_message):
        pose = odom_message.pose.pose
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]

    def get_odom(self):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        if self.current_odom == None:
            return None, None, None
        return BaseLidar.convert_odom(self.current_odom)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # node = ReceiveLidar()
    # while not rospy.is_shutdown():
    #     print(node.get_wall())
    #     rospy.sleep(.3)
    node = SendLineMarker()
    while not rospy.is_shutdown():
        node.update_marker([(1,-1),(1,0),(1,1), (.3,.5)])
    # rospy.spin()
