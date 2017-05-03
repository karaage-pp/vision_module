#!/usr/bin/env python
# -*- coding: utf-8 -*-
import alexnet as net
import numpy

classifier = net.Alexnet()

def setup(pretrained_model):
    classifier.setup(pretrained_model)
    return 0

def classify(img, width, height):
    return classifier.getProbResult()

def extractFeatures(img, width, height):
    image = numpy.array(img, dtype = 'uint8').reshape((height, width, 3))
    return classifier.extractFeatures(image)

