#!/usr/bin/env python
import sys
sys.path.append('./../face_prediction/')
import rospy
import rospkg
from sensor_msgs.msg import Image
from neato_node.msg import Bump
from cv_bridge import CvBridge
import cPickle as pickle
import numpy as np
import cv2
import detect_people
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt

MODEL_PATH = '/Data/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'
MAX_TURN = .46 #Used for conversion from pixels to turning speed
MAX_MOVE = .2 #Used for conversion betweeen pixels and forward speed
MAX_PIXELS = 240 #Makes difference in pixels between -1 and 1
IMAGE_SIZE = 448,256,3 #Input image size
AREA = IMAGE_SIZE[0] * IMAGE_SIZE[1]


class TrackPerson:
    def __init__(self):
        rospy.init_node('person_tracker')
        print('init')
        self.turn_velocity = 0 #start turn velocity
        self.forward_velocity = 0 #start forward velocity
        self.threshold_person = 0.5 #How sure the pre-trained model is that there's a person in screen
        self.threshold_direction = 10 #in pixels, how far the person's bounding box is to decide to turn
        self.bridge = CvBridge() #used to get images
        self.get_image = False #whether to graab newest image
        self.image = None #newest image
        self.model = detect_people.DetectorAPI(path_to_ckpt = MODEL_PATH) #Load person detector model
        rospy.Subscriber('camera/image_raw', Image, self.process_image) #Camera subscriber

    def process_image(self, m):
        '''Decide whether or not to save the most recent message. We use the bool
        self.get_image so that we don't spend the compute of saving every image,
        rather only when we're ready for another image'''
        if self.get_image:
            self.image = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
            self.get_image = False

    def get_box(self):
        '''Return a bounding box if there's a person in the image '''
        boxes, scores, classes, num = self.model.processFrame(self.image) #check for people
        x_max = self.image.shape[0]
        classes=np.array(classes)
        scores = np.array(scores)
        classes_index = np.where(classes == 1)[0] #find all guesses that predict people
        if len(classes_index):
            #find the guess that has the most confidence that it's a person
            max_person = np.argmax(scores[classes_index])
            max_person_index = classes_index[max_person]
            if scores[max_person_index] > self.threshold_person:
                #if the max confiednce is over a threshold return the max box
                return boxes[max_person_index]
        return None

    def set_speed(self, box):
        '''Set proportional Control based on where the bounding box is'''
        x_max = self.image.shape[0]

        rectange_center = (box[1] + box[3])//2 #get center of box
        box_dir = float(x_max//2 - rectange_center) #find where center of box is in relation to center of image
        if abs(box_dir) > self.threshold_direction:
            #Set a proportional velocity if the bounding box is enough to the side
            self.turn_velocity=MAX_TURN * box_dir/MAX_PIXELS
            self.forward_velocity = MAX_MOVE * (1- abs(box_dir/MAX_PIXELS))
        elif abs(box_dir) <= self.threshold_direction:
            #Full steam ahead if bounding box is in center
            self.turn_velocity = 0
            self.forward_velocity = MAX_MOVE

    def decide_stop(box):
        x = box[3] - box[1]
        y = box[2] - box[0]
        if abs(x * y) > (AREA//2):
            self.forward_velocity = 0

if __name__ == "__main__":
    node = TrackPerson()
    node.run()
