
import tensorflow as tf
import sys
import os
import numpy as np
import cv2
import time
from model import Model
from network import *
from datetime import datetime

n_classes = 1000
crop_size = 227
scale_size = 256

class Alexnet:
    def __init__(self):
        gpuConfig = tf.ConfigProto(
            gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction = 0.5))
        self.x = tf.placeholder(tf.float32, [1, crop_size, crop_size, 3])
        self.pred, self.features = Model.alexnet_fe(self.x, 1.0, n_classes)
        self.prob = tf.nn.softmax(self.pred)
        self.sess = tf.Session(config = gpuConfig)
        self.saver = tf.train.Saver()

        self.prob_result = None

    def __del__(self):
        self.sess.close()

    def setup(self, pretrained_model):
        self.saver.restore(self.sess, pretrained_model)

    def extractFeatures(self, image):
        reshaped = self.reshape(image)
        feat, self.prob_result = self.sess.run([self.features, self.prob], feed_dict={self.x: reshaped})
        return feat.reshape((-1)).tolist()

    def getProbResult(self):
        return self.prob_result.reshape((-1)).tolist()

    def classify(self, image):
        reshaped = self.reshape(image)
        prob = self.sess.run(self.prob, feed_dict={self.x: reshaped})
        return prob.reshape((-1)).tolist()

    def reshape(self, img):
        mean = np.array([104., 117., 124.])

        h, w, c = img.shape
        assert c==3

        reshaped = np.ndarray([1, crop_size, crop_size, 3])
        resized = cv2.resize(img, (scale_size, scale_size))
        resized = resized.astype(np.float32)
        resized -= mean
        shift = int((scale_size - crop_size) / 2)
        resized = resized[shift:shift + crop_size, shift:shift + crop_size, :]
        reshaped[0] = resized

        return reshaped


def main(argv):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "input_file",
        help = "Input image or image list."
    )
    parser.add_argument(
        "--pretrained_model",
        default = "../model/bvlc_reference_caffenet.tf",
        help = "Pretrained model."
    )
    parser.add_argument(
        "--output_file",
        default = "features.txt",
        help = "Output file name."
    )
    parser.add_argument(
        "--extract",
        action = "store_true",
        help = "Extract features from dataset"
    )
    args = parser.parse_args()
    args.input_file = os.path.expanduser(args.input_file)

    net = Alexnet()
    net.setup(args.pretrained_model)

    if args.extract:
        f = open(args.input_file)
        lines = f.readlines()
        f.close()

        with open(args.output_file, "a") as f:
            for line in lines:
                data = line.split(" ")
                label = data[0]
                fname = data[1].strip()
                image = cv2.imread(fname)
                print "extracting features from " + fname
                feats = net.extractFeatures(image)
                f.write(label)
                i = 1
                for feat in feats:
                    ss = "\t{0}:{1:.6f}".format(i, feat)
                    f.write(ss)
                    i = i + 1
                f.write("\n")

    else:
        image = cv2.imread(args.input_file)

        start = time.time()

        feat = net.classify(image)
        np.savetxt(args.output_file, feat, fmt = '%.6f', delimiter='\n')

        end = time.time()
        print 'Time: ' + str(end - start) + "(secs)"

if __name__ == '__main__':
    import argparse

    main(sys.argv)
