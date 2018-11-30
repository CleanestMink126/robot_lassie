import numpy as np
from math import sin, cos, atan2, pi, fabs
import rospy
import cv2
import interface
import person_tracking
import mapLogic
print('Imported')

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return atan2(sin(z), cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2

rad2degrees = 180/pi
degrees2rad = pi/180

class TrackPath(object):
    '''This class will handle finding a person who moves directly in front of
    the neato, then tracking their position and following their path
    '''
    def __init__(self):
        self.points = [] #list of tuples representing positions
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.my_marker = interface.SendMarker()
        self.my_bump = interface.ReceiveBump()
        self.person_tracker = person_tracking.TrackPerson()
        x = None
        while x is None:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
        self.map = mapLogic.Map((x,y,yaw))
        self.thres = .25 #how close can a point be before the neato ignores it
        self.speed = .3 #how fast to move

    def update_map(self):
        self.my_lidar.get_lidar = True
        while self.my_lidar.get_lidar:
            rospy.sleep(.05)
        self.map.update_graph(self.my_lidar.last_odom, self.my_lidar.last_ranges)
        self.my_lidar.reset()

    def show_map(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.map.update_map()
            cv2.imshow("image",self.map) #show box
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                self.my_speed.send_speed(0,0)
                break
            r.sleep()


    def add_position(self):
        '''Add a point to the list of points to follow'''
        x, y, yaw = self.my_lidar.my_odom.get_odom()
        self.points.append((x,y))

    def navigate_to_point(self):
        '''If we reached a point, turn to the next point in the list and
        drive there'''
        self.my_speed.send_speed(0,0)
        x, y, yaw = self.my_lidar.my_odom.get_odom()
        while distance(self.points[0],(x,y)) < self.thres:
            self.points.pop(0)#ignore redundant points that are close to neato
            if not len(self.points):
                self.my_speed.send_speed(0,0)
                return
        angle_threshold = pi / 38 #how close we need to be in our heading to next point
        c_angle_diff = angle_threshold + 1 #just so the while loop will run
        while abs(c_angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.points[0][0], self.points[0][1]
            t_yaw = atan2(y_t - y, x_t - x) #calculate ideal angle
            c_angle_diff = angle_diff(t_yaw, yaw) #find difference from current angle
            #effectively turn but don't stop (with some hard coded numbers we found)
            self.my_speed.send_speed(.1-abs(c_angle_diff/(10*pi)), np.sign(c_angle_diff) * .5 +  c_angle_diff/2)
        self.my_speed.send_speed(self.speed,0)#go forward

    def check_progress(self):
        '''Loop to check if we have either passed our point or are close to it'''
        if len(self.points):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.points[0][0], self.points[0][1]
            t_yaw = atan2(y_t - y, x_t - x)
            c_angle_diff = angle_diff(t_yaw, yaw)
            if distance(self.points[0],(x,y)) < self.thres or abs(c_angle_diff)>pi/2:
                #We passed the point
                return True
        elif not len(self.points):#stop if we run out of points
            self.my_speed.send_speed(0,0)
        return False

    def retrace_path(self):
        '''Mainloop to control robot'''
        r= rospy.Rate(5)
        self.points = self.points[::-1]
        print(self.points)
        self.navigate_to_point()
        while not rospy.is_shutdown():#add the persons position for a set amount of time
            if not len(self.points):
                self.my_speed.send_speed(0,0)
                break
            # self.my_marker.update_marker(self.points, frame_id = 'odom')
            if self.check_progress() and len(self.points):
                #loop to navigate to points
                self.navigate_to_point()
            r.sleep()
        print('Back to start')

    def go_to_person(self):
        '''This is the main loop for the Neato to follow a person infront of it'''
        r = rospy.Rate(3)#How fast to run the loop
        while not rospy.is_shutdown():
            self.my_speed.send_speed(self.person_tracker.forward_velocity, self.person_tracker.turn_velocity)#start off by sending the current speed
            self.add_position()
            if self.person_tracker.image is not None: #if we have gotten an image
                box = self.person_tracker.get_box()
                if box is None:#if we did not find a suitable box, try again later
                    self.person_tracker.reset()
                    self.person_tracker.get_image = True
                    continue
                # cv2.imshow("image",img) #show box
                # key = cv2.waitKey(1)
                # if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                #     self.my_speed.send_speed(0,0)
                #     break
                self.person_tracker.set_speed(box) # determine proportional speed
                if self.person_tracker.decide_stop(box):
                    break #determine whether or not to stop
                self.person_tracker.reset() #make sure to wait for a new image
            self.person_tracker.get_image = True
            r.sleep()
        print('Finished go to person')

    def explore(self):
        r = rospy.Rate(3)#How fast to run the loop
        self.my_speed.send_speed(0, .1)#start off by sending the current speed
        while not rospy.is_shutdown():
            if self.person_tracker.image is not None: #if we have gotten an image
                box = self.person_tracker.get_box()
                cv2.imshow("image",self.person_tracker.image) #show box
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                    self.my_speed.send_speed(0,0)
                    break
                self.person_tracker.reset() #make sure to wait for a new image
                self.person_tracker.get_image = True
                if box is not None:#if we did not find a suitable box, try again later
                    break
            self.person_tracker.get_image = True
            r.sleep()
        print('Found person')

if __name__ == "__main__":
    print('Start')
    tracker = TrackPath()
    tracker.show_map()
    # tracker.explore()
    # tracker.go_to_person()
    # tracker.retrace_path()
