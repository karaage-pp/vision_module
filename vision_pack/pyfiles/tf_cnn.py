
import sys
import tensorflow as tf
import numpy as np
import cv2
from model import Model
from network import *


class TF_CNN(object):
    def load_model(self, pretrained_model, skip_list=["None"]):
        extension = pretrained_model.split(".")[1]
        if extension == "tf":
            self.saver.restore(self.sess, pretrained_model)
        elif extension == "npy":
#            init = tf.global_variables_initializer()
#            self.sess.run(init)
            load_with_skip(pretrained_model, self.sess, skip_list)

    def save_model(self, model_file_name):
        save_path = self.saver.save(self.sess, model_file_name)
        return save_path

    def reshape(self, img, crop_size, scale_size):
        mean = np.array([104., 117., 124.])

        h, w, c = img.shape

        assert c==3

        reshaped = np.ndarray([1, crop_size, crop_size, 3])
        resized = cv2.resize(img, (scale_size, scale_size))

        resized = resized.astype(np.float32)
        resized -= mean
        shift = int((scale_size - crop_size) / 2)
        resized = resized[
                shift:shift + crop_size,
                shift:shift + crop_size, :]
        reshaped[0] = resized
        return reshaped
