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
SHOW_MAP = True

class TrackPath(object):
    '''This class will handle all the navigation and high level commands for the
    Neato
    '''
    def __init__(self):
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.my_marker = interface.SendMarker()
        self.person_tracker = person_tracking.TrackPerson()
        x = None
        while x is None:#Get an initial read for the poition of the neato
            x, y, yaw = self.my_lidar.get_odom()
        print('Setup')#use that position to define a new map
        self.map = mapLogic.Map((x,y,yaw))
        print('Made map')
        self.thres = .15 #how close can a point be before the neato ignores it
        self.speed = .2 #how fast to move

    def update_map(self):
        '''Indicate to the LIDAR scanner to hold onto the next value and odom
        at that time'''
        self.my_lidar.get_lidar = True
        while self.my_lidar.get_lidar:
            rospy.sleep(.05)
        self.map.update_graph(self.my_lidar.last_odom, self.my_lidar.last_ranges)
        self.my_lidar.reset()#Reset LIDAR to not look for values

    def vis_map(self):
        ''''This is just a quick visualization to make sure the mapping is
        working when manually driving the neato around'''
        r = rospy.Rate(2)
        last = rospy.get_rostime().to_sec()
        x_goal, y_goal = 0, 0
        dist_map = np.zeros_like(self.map.graph[:,:,0])
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output.avi',fourcc, 6.0, (self.map.size,self.map.size))
        while not rospy.is_shutdown():
            self.update_map()
            new_graph = np.copy(self.map.graph)
            #------------------------
            now = rospy.get_rostime().to_sec()
            if now - last > 15:
                last = now
                _, goal_loc, dist_map = self.map.set_goal()
                x_goal, y_goal = int(goal_loc[0]), int(goal_loc[1])
            new_graph[x_goal-1:x_goal+1,y_goal-1:y_goal+1,:] = [255,255,255]
            x_val, y_val = np.where(dist_map)
            new_graph[x_val, y_val,0] += (dist_map[x_val, y_val]/20.0 * new_graph[x_val, y_val,1]).astype(np.uint8)
            out.write(new_graph)
            cv2.imshow("image",new_graph) #show box
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                self.my_speed.send_speed(0,0)
                break
            r.sleep()

    def navigate_to_point(self,point):
        '''Use some simple proportional controls to get to a point'''
        angle_threshold = pi / 38 #how close we need to be in our heading to next point
        c_angle_diff = angle_threshold + 1 #just so the while loop will run
        while abs(c_angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.get_odom()
            x_t, y_t = point[0], point[1]
            t_yaw = atan2(y_t - y, x_t - x) #calculate ideal angle
            c_angle_diff = angle_diff(t_yaw, yaw) #find difference from current angle
            #effectively turn but don't stop (with some hard coded numbers we found)
            self.my_speed.send_speed(.1-abs(c_angle_diff/(10*pi)), np.sign(c_angle_diff) * .5 +  c_angle_diff/2)
        self.my_speed.send_speed(self.speed,0)#go forward

    def time_to_point(self, point):
        '''Use more exact measurements to get to a point'''
        x, y, yaw = self.my_lidar.get_odom()
        x_t, y_t = point[0], point[1]
        t_yaw = atan2(y_t - y, x_t - x) #calculate ideal angle
        c_angle_diff = angle_diff(t_yaw, yaw) #find difference from current angle
            #effectively turn but don't stop (with some hard coded numbers we found)
        self.my_speed.send_speed(.05, 1 * np.sign(c_angle_diff))
        rospy.sleep(abs(c_angle_diff))
        self.my_speed.send_speed(self.speed,0)#go forward


    def find_next_point(self, points):
        '''If we reached a point, turn to the next point in the list and
        drive there'''
        self.my_speed.send_speed(0,0)
        x, y, yaw = self.my_lidar.get_odom()
        if not points.shape[0]:
            return False
        while distance(points[0],(x,y)) < self.thres:
            points = np.delete(points,0,0)#ignore redundant points that are close to neato
            if not points.shape[0]:
                return False
        self.time_to_point(points[0])
        return True


    def check_progress(self, point):
        '''Loop to check if we have either passed our point or are close to it'''
        x, y, yaw = self.my_lidar.get_odom()
        x_t, y_t = point[0], point[1]
        t_yaw = atan2(y_t - y, x_t - x)
        c_angle_diff = angle_diff(t_yaw, yaw)
        if distance(point,(x,y)) < self.thres or abs(c_angle_diff)>pi/2:
            #We passed the point
            return True
        return False

    def return_to_start(self):
        self.spin()
        self.goal_path, self.goal, self.weighted_map = self.map.set_goal(self.map.center)
        self.goal_path = self.goal_path[::3]
        self.find_next_point(self.goal_path)
        while not rospy.is_shutdown():
            self.update_map()
            if self.check_progress(self.goal_path[0]):
                self.goal_path = np.delete(self.goal_path,0,0)
                print('Reached', self.goal_path)
                self.find_next_point(self.goal_path)
                if not len(self.goal_path):
                    break
        self.spin()

    def spin(self):
        self.my_speed.send_speed(0, 3.14/2.0)
        rospy.sleep(4)
        self.my_speed.send_speed(0, 0)

    def go_to_person(self):
        '''This is the main loop for the Neato to follow a person infront of it'''
        r = rospy.Rate(4)#How fast to run the loop
        while not rospy.is_shutdown():
            self.my_speed.send_speed(self.person_tracker.forward_velocity, self.person_tracker.turn_velocity)#start off by sending the current speed
            self.update_map()
            if self.person_tracker.image is not None: #if we have gotten an image
                box = self.person_tracker.get_box()
                if box is None:#if we did not find a suitable box, try again later
                    self.person_tracker.reset()
                    self.person_tracker.get_image = True
                    continue
                self.person_tracker.set_speed(box) # determine proportional speed
                if self.person_tracker.decide_stop(box):
                    break #determine whether or not to stop
                self.person_tracker.reset() #make sure to wait for a new image
            self.person_tracker.get_image = True
            r.sleep()
        print('Finished go to person')

    def explore_static(self,num_maps=30, speed= .5):
        self.my_speed.send_speed(0, speed)#start off by sending the current speed
        for i in range(num_maps):
            if self.person_tracker.image is not None: #if we have gotten an image
                #------------------Person Tracking
                box = self.person_tracker.get_box()
                self.person_tracker.reset() #make sure to wait for a new image
                self.person_tracker.get_image = True
                if box is not None:#if we did not find a suitable box, try again later
                    return True
            else:
                self.person_tracker.get_image = True
            self.update_map()
            cv2.imshow("image",self.map.graph) #show box
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                self.my_speed.send_speed(0,0)
                break
        self.goal_path, self.goal, self.weighted_map = self.map.set_goal()
        self.goal_path = self.goal_path[::3]
        return False

    def explore(self):
        r = rospy.Rate(3)#How fast to run the loop
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output.avi',fourcc, 10.0, (self.map.size,self.map.size))
        self.explore_static()
        self.find_next_point(self.goal_path)
        # num_goals = 0
        while not rospy.is_shutdown():
            if self.person_tracker.image is not None: #if we have gotten an image
                #------------------Person Tracking
                box = self.person_tracker.get_box()
                self.person_tracker.reset() #make sure to wait for a new image
                self.person_tracker.get_image = True
                if box is not None:#if we did not find a suitable box, try again later
                    break
            else:
                self.person_tracker.get_image = True
            #-------------------
            self.update_map()
            if SHOW_MAP:
                x_goal = int(self.goal[0])
                y_goal = int(self.goal[1])
                new_graph = np.copy(self.map.graph)
                new_graph[x_goal-1:x_goal+1,y_goal-1:y_goal+1,:] = [255,255,255]
                x_val, y_val = np.where(self.weighted_map)
                new_graph[x_val, y_val,0] += (self.weighted_map[x_val, y_val]/20.0 * new_graph[x_val, y_val,1]).astype(np.uint8)
                cv2.imshow("image",new_graph) #show box
                out.write(new_graph)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'): #stop neato and quit if 'q'
                    self.my_speed.send_speed(0,0)
                    break
            if self.check_progress(self.goal_path[0]):
                self.goal_path = np.delete(self.goal_path,0,0)
                print('Reached', self.goal_path)
                self.find_next_point(self.goal_path)
                while not len(self.goal_path):
                    # num_goals += 1
                    # if num_goals >= 3:
                    #     return
                    if self.explore_static():
                        return
                    self.find_next_point(self.goal_path)
            r.sleep()
        print('Found person')

if __name__ == "__main__":
    print('Start')
    tracker = TrackPath()
    # tracker.vis_map()
    tracker.explore()
    tracker.go_to_person()
    tracker.return_to_start()
