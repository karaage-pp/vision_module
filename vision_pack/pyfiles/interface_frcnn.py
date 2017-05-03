
from frcnn import FRCNN
import numpy

net = FRCNN()

def setup(model_name):
    net.set_model(model_name)
    return 0

def detect_object(img, width, height):
    image = numpy.array(img, dtype = 'uint8').reshape((height, width, 3))
    result = net.detect_object(image)
    return result
    
