
import tensorflow as tf
import sys
import os
import argparse
import numpy as np
import cv2
import time
from tf_cnn import TF_CNN
from model import Model
from network import *
from datetime import datetime

n_classes = 2622
scale_size = 256

def DEL(self):
    self.sess.close()

class FACE_NET(TF_CNN):
    def __init__(self):
        gpuConfig = tf.ConfigProto(
            gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction = 0.5))
        self.x224 = tf.placeholder(tf.float32, [1, 224, 224, 3])
        self.x227 = tf.placeholder(tf.float32, [1, 227, 227, 3])
        self.pred, self.features = Model.VGG_NET(self.x224, 1.0, n_classes)
        self.person = tf.nn.softmax(self.pred)
        self.age = tf.nn.softmax(Model.AGE_NET(self.x227, 1.0))
        self.gender = tf.nn.softmax(Model.GENDER_NET(self.x227, 1.0))

        self.sess = tf.Session(config = gpuConfig)
        self.saver = tf.train.Saver()
        self.init = tf.initialize_all_variables()

        super(FACE_NET, self).__init__()

    def __del__(self):
        DEL(self)

    def extractFeatures(self, image):
        reshaped = self.reshape(image, 224, scale_size)
        print reshaped.shape
        init = tf.initialize_all_variables()
        self.sess.run(init)
        feat = self.sess.run(self.features, feed_dict={self.x224: reshaped})
        return feat.reshape((-1)).tolist()

    def who(self, image):
        reshaped = self.reshape(image, 224, scale_size)
        prob = self.sess.run(self.person, feed_dict={self.x224: reshaped})
        return prob.reshape((-1)).tolist()

    def how(self, image):
        reshaped = self.reshape(image, 227, scale_size)
        prob = self.sess.run(self.age, feed_dict={self.x227: reshaped})
        return prob.reshape((-1)).tolist()

    def which(self, image):
        reshaped = self.reshape(image, 227, scale_size)
        prob = self.sess.run(self.gender, feed_dict={self.x227: reshaped})
        return prob.reshape((-1)).tolist()

class VGG_NET(TF_CNN):
    def __init__(self):
        self.x = tf.placeholder(tf.float32, [1, 224, 224, 3])
        self.pred, self.features = Model.VGG_NET(self.x, 1.0, n_classes)
        self.prob = tf.nn.softmax(self.pred)

        self.sess = tf.Session()
        self.saver = tf.train.Saver()
        super(VGG_NET, self).__init__()

    def __del__(self):
        DEL(self)

    def extractFeatures(self, image):
        reshaped = self.reshape(image, crop_size, scale_size)
        feat = self.sess.run(self.features, feed_dict={self.x: reshaped})
        return feat.reshape((-1)).tolist()

    def classify(self, image):
        reshaped = self.reshape(image, crop_size, scale_size)
        prob = self.sess.run(self.prob, feed_dict={self.x: reshaped})
        return prob.reshape((-1)).tolist()

class AGE_NET(TF_CNN):
    def __init__(self):
        self.x = tf.placeholder(tf.float32, [1, 227, 227, 3])
        self.age = tf.nn.softmax(Model.AGE_NET(self.x, 1.0))

        self.sess = tf.Session()
        self.saver = tf.train.Saver()
        super(AGE_NET, self).__init__()

    def __del__(self):
        DEL(self)

    def classify(self, image):
        reshaped = self.reshape(image, 227, 227)
        prob = self.sess.run(self.age, feed_dict={self.x: reshaped})
        return prob.reshape((-1)).tolist()

class GENDER_NET(TF_CNN):
    def __init__(self):
        self.x = tf.placeholder(tf.float32, [1, 227, 227, 3])
        self.gender = tf.nn.softmax(Model.GENDER_NET(self.x, 1.0))

        self.sess = tf.Session()
        self.saver = tf.train.Saver()
        super(GENDER_NET, self).__init__()

    def __del__(self):
        DEL(self)

    def classify(self, image):
        reshaped = self.reshape(image, 227, 227)
        prob = self.sess.run(self.gender, feed_dict={self.x: reshaped})
        return prob.reshape((-1)).tolist()

def main(argv):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input_image",
        default = "~/Pictures/Adrian_Paul.jpg",
        help = "Input image."
    )
    parser.add_argument(
        "--face_model",
        default = "model/face_net.tf",
        help = "Pretrained age model."
    )
    args = parser.parse_args()
    args.input_image = os.path.expanduser(args.input_image)

    net = FACE_NET()
    net.load_model("model/face_net.tf")

    image = cv2.imread(args.input_image)

    start = time.time()

    feat = net.extractFeatures(image)
    who = net.who(image)
    how = net.how(image)
    which = net.which(image)

    end = time.time()

    whoResult = "result/who.txt"
    howResult = "result/how.txt"
    whichResult = "result/which.txt"
    np.savetxt("result/feat.txt", feat, fmt = '%.8f', delimiter='\t')
    np.savetxt(whoResult, who, fmt = '%.8f', delimiter='\n')
    np.savetxt(howResult, how, fmt = '%.8f', delimiter='\n')
    np.savetxt(whichResult, which, fmt = '%.8f', delimiter='\n')

    print 'Time: ' + str(end - start) + "(secs)"

if __name__ == '__main__':
    main(sys.argv)
