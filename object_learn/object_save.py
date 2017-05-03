# !/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import math
import time

import rospy
import numpy as np
from PIL import Image
from tqdm import *
from vision_module.msg import ObjectInfo
PATH = os.path.expanduser("~") + "/Documents/TrainImage/"
NUM_IMAGE_TO_SAVE = 5

class SaveTrainImage():
    '''Class subcsribe to object_detection_info and save RGB image'''
    def __init__(self):
        self.average_point = np.array((0.0, 0.0))
        self.obj_cnt = 0
        self.saved_num = 0
        self.message = None

    def get_message(self, message):
        '''Get current message of topic'''
        self.message = message
        return self.message

    def get_xy_position(self, obj_index):
        '''Get xy position of object with index of obj_index'''
        x_pos = self.message.objects[obj_index].camera.x
        y_pos = self.message.objects[obj_index].camera.y
        xy_array = np.array((x_pos, y_pos))
        return xy_array

    def command_input(self, command_str):
        '''Command to control the saving process'''
        stop = False
        while not stop:
            ans = raw_input(command_str + "[y/n] ")
            if ans == "n":
                sys.exit()
            elif ans == "y":
                stop = True
            else:
                print "Invalid answer! Try again...\n"
        return None

    def make_image_dir(self, obj_no):
        '''Create dir for saving image if not exist'''
        path = PATH + "ObjNo_{}/".format(obj_no)
        if not os.path.exists(path):
            os.makedirs(path)
        return None

    def get_save_image_index(self):
        '''Get the index of image to save'''
        dist_to_avg = []
        for obj_index in range(len(self.message.objects)):
            position = self.get_xy_position(obj_index)
            dist_to_avg.append(np.linalg.norm(position - self.average_point))
        dist_to_avg = np.array(dist_to_avg)
        save_obj_index = np.argmin(dist_to_avg)
        return save_obj_index

    def save_image(self, message, index, img_name):
        '''Convert uint8 string image to png image and save to PATH'''
        width = message.objects[index].width
        height = message.objects[index].height
        uint8_data = np.fromstring(message.objects[index].bgr, dtype=np.uint8)
        uint8_data = uint8_data.reshape((height, width, 3))
        img = Image.fromarray(uint8_data, "RGB")
        img.save(PATH + str(img_name) + ".png")
        return None

    def record_train_image(self):
        '''Main program for saving process'''
        self.command_input("Start saving training image? ")
        while not rospy.is_shutdown():
            self.command_input("Object setting OK? ")
            self.obj_cnt += 1
            sub = rospy.Subscriber('/vision_module/object_detection_info',
                                   ObjectInfo, self.get_message)
            time.sleep(1)
            init_xy = self.get_xy_position(0)
            self.average_point = (self.average_point + init_xy) / self.obj_cnt
            for i in tqdm(range(NUM_IMAGE_TO_SAVE)):
                self.saved_num = i
                self.make_image_dir(self.obj_cnt)
                save_obj_index = self.get_save_image_index()
                self.save_image(self.message, save_obj_index,\
                                "ObjNo_{}/imageNo_{}".format(self.obj_cnt, self.saved_num))
                time.sleep(1)
            self.command_input("Continue with other object? ")

if __name__ == "__main__":
    rospy.init_node('save_object_image')
    saveTrainImage = SaveTrainImage()
    saveTrainImage.record_train_image()


