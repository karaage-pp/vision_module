#!/usr/bin/env python
# -*- coding: utf-8 -*-
from face import FACE_NET
import numpy

net = FACE_NET()

def setup(model):
    net.load_model(model)
    return 0

def extractFeatures(img, width, height):
    # f = open("test.txt", "w")
    # for item in img:
    #     f.write("%d," % item)
    # f.close()
    image = numpy.array(img)
    image = image.astype(numpy.float32)
    #image = image.astype(numpy.uint8)
    image = image.reshape((height, width, 3))
    # feat = net.extractFeatures(image)
    feat = net.extractFeatures(image)
    return feat

def who(img, width, height):
    image = numpy.array(img, dtype = 'uint8').reshape((height, width, 3))
    return net.who(image)

def how(img, width, height):
    image = numpy.array(img, dtype = 'uint8').reshape((height, width, 3))
    return net.how(image)

def which(img, width, height):
    image = numpy.array(img, dtype = 'uint8').reshape((height, width, 3))
    return net.which(image)
