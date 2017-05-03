import argparse
import os
import sys
import shutil

import rospy
import numpy as np
from PIL import Image
from tqdm import *
import cv2
from vision_module.msg import ObjectInfo


NUM_TO_SAVE = 1000

class SaveTrainImage():
    def __init__(self, folder, num_to_save, image_id, obj_id):
        self.save_id = int(image_id)
        self.obj_id = obj_id
        self.PATH = os.path.expanduser('~') + \
                    "/Documents/{}/{}/".format(folder, self.obj_id)
        self.IMAGE_NUM =  int(num_to_save)
        self.firstID_flag = True
        self.last_pos = None
        self.stop = False
        sub = rospy.Subscriber('/vision_module/object_detection_info',
                               ObjectInfo, self.callback)
        self.make_dir()

    def get_xy_position(self, message):
        '''Get xy position of object with index of obj_index'''
        x_pos = message.objects[0].camera.x
        y_pos = message.objects[0].camera.y
        xy_array = np.array((x_pos, y_pos))
        return xy_array

    def command_input(self, command_str):
        '''Command to control the saving process'''
        stop = False
        while not stop:
            ans = raw_input(command_str + "[y/n] ")
            if ans == "n":
                self.stop = True
                sys.exit()
            elif ans == "y":
                stop = True
            else:
                print "Invalid answer! Try again...\n"
        return stop

    def make_dir(self):
        if os.path.exists(self.PATH):
            if os.path.isfile(self.PATH + "/" + "{0:03}".format(self.save_id) + ".jpg"):
                stop = self.command_input("Files with this name already exist! Remove entire folder? ")
                if stop:
                    shutil.rmtree(self.PATH)
        if not os.path.exists(self.PATH):
            os.makedirs(self.PATH)
        return None

    def save_image(self, message, index, img_name, mode="test"):
        '''Convert uint8 string image to png image and save to PATH'''
        width = message.objects[index].width
        height = message.objects[index].height
        uint8_data = np.fromstring(message.objects[index].bgr, dtype=np.uint8)
        uint8_data = uint8_data.reshape((height, width, 3))
        uint8_revert = cv2.cvtColor(uint8_data, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(uint8_revert, "RGB")
        img.save(self.PATH + img_name + ".jpg")
        if mode=="save":
            self.save_id += 1


    def callback(self, message):
        if self.firstID_flag == True:
            self.last_pos = self.get_xy_position(message)
            # self.save_image(message, 0, "test")

        if self.firstID_flag == False:
            dist_to_last = []
            for obj_index in range(len(message.objects)):
                position = self.get_xy_position(message)
                dist_to_last.append(np.linalg.norm(position - self.last_pos))
            dist_to_last = np.array(dist_to_last)
            if dist_to_last == None:
                return
            save_obj_index = np.argmin(dist_to_last)
            self.save_image(message, save_obj_index, "{0:03}".format(self.save_id), "save")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Parameters setting')
    parser.add_argument('-d', '--save_directory', default="TrainImage")
    parser.add_argument('-n', "--save_image_numbers", default=NUM_TO_SAVE, required=False)
    parser.add_argument('-id', "--beginning_image_id", default=0, required=False)
    parser.add_argument('-o', "--object_index", required=True)
    args = parser.parse_args()

    rospy.init_node("save_object_image")
    saveTrainImage = SaveTrainImage(args.save_directory,
                                    args.save_image_numbers,
                                    args.beginning_image_id,
                                    args.object_index)
    saveTrainImage.command_input("Setting object No {} OK? ".format(saveTrainImage.obj_id))
    while not rospy.is_shutdown():
        saveTrainImage.firstID_flag = False
        print "Saving image No {}/{}".format(saveTrainImage.save_id, args.save_image_numbers)
        sys.stdout.write("\033[F")
        if saveTrainImage.save_id > (int(args.beginning_image_id) + saveTrainImage.IMAGE_NUM - 1):
            sys.exit()
