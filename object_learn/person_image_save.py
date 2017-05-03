import argparse
import os
import sys
import shutil

import rospy
import numpy as np
from PIL import Image

from vision_module.msg import ObjectInfo
import cv2
try:
    import tqdm
except:
    print ("Cant import tqdm module")

class SavePersonImage():
    def __init__(self, folder, num_to_save, pose_id):
        self.save_id = 0
        self.pose_id = pose_id
        if folder == None:
            self.PATH = os.path.expanduser('~') + \
                    "/Documents/person/{}/".format(self.pose_id)
        else:
            self.PATH = folder + "/{}/".format(self.pose_id)
        self.IMAGE_NUM =  num_to_save
        self.stop = False
        sub = rospy.Subscriber('/vision_module/object_detection_info',
                               ObjectInfo, self.callback)
        self.firstID_flag = True
        self.make_dir()

    def get_person_index(self, message):
        '''Get index of person'''
        person_index = None
        max_score = 0
        for i in xrange(len(message.objects)):
            if message.objects[i].generic[0].name == "person":
                if max_score < message.objects[i].generic[0].score:
                    max_score = message.objects[i].generic[0].score
                    person_index = i
            # print (message.objects[i].generic[0].name)
        return person_index 

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
        return None

    def make_dir(self):
        if os.path.exists(self.PATH):
            self.command_input("Folder already exist! Remove? ")
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
        
        img.save(self.PATH + img_name + ".png")
        if mode=="save":
            self.save_id += 1
        return None

    def callback(self, message):
        if self.firstID_flag == True:
            person_index = self.get_person_index(message)
            self.save_image(message, person_index, "test")
        
        if self.firstID_flag == False:
            person_index = self.get_person_index(message)
            self.save_image(message, person_index, "imgID_{}".format(self.save_id), "save")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Parameters setting')
    parser.add_argument('-d', '--save_directory', default=None)
    parser.add_argument('-n', "--save_image_numbers", default=1000, required=False)
    parser.add_argument('-p', "--pose_index", required=True)
    args = parser.parse_args()

    rospy.init_node("save_person_image")
    savePersonImage = SavePersonImage(args.save_directory,
                                    args.save_image_numbers,
                                    args.pose_index)
    savePersonImage.command_input("Person pose No {} OK? ".format(savePersonImage.pose_id))
    while not rospy.is_shutdown():
        savePersonImage.firstID_flag = False
        print "Saving image No {}/100".format(savePersonImage.save_id)
        sys.stdout.write("\033[F")
        if savePersonImage.save_id > args.save_image_numbers:
            sys.exit()


