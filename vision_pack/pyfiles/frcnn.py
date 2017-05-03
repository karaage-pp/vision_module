#!/usr/bin/env python
# -*- coding: utf-8 -*-

from chainer import serializers
from chainer.cuda import to_gpu
from lib.cpu_nms import cpu_nms as nms
from lib.models.faster_rcnn import FasterRCNN

import chainer
import cv2 as cv
import numpy as np

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')
PIXEL_MEANS = np.array([[[102.9801, 115.9465, 122.7717]]])

class FRCNN:
    def __init__(self, gpu=-1):
        self.gpu = gpu
        self.model = None

    def set_model(self, model_name):
        model = FasterRCNN(self.gpu)
        model.train = False
        serializers.load_npz(model_name, model)
        self.model = model
        if chainer.cuda.available and self.gpu >= 0:
            self.model.to_gpu(self.gpu)

    def detect_object(self, input_image, 
            nms_thresh = 0.3, conf = 0.8, draw = False):

        img, im_scale = self.img_preprocessing(input_image, PIXEL_MEANS)
        img = np.expand_dims(img, axis=0)
        if self.gpu >= 0:
            img = to_gpu(img, device=self.gpu)
        img = chainer.Variable(img, volatile=True)
        h, w = img.data.shape[2:]
        cls_score, bbox_pred = self.model(img, np.array([[h, w, im_scale]]))
        cls_score = cls_score.data

        if self.gpu >= 0:
            cls_score = chainer.cuda.cupy.asnumpy(cls_score)
            bbox_pred = chainer.cuda.cupy.asnumpy(bbox_pred)
        op = self.draw_result if draw else self.get_dictionary
        result = op(input_image, im_scale, cls_score, bbox_pred, 
                nms_thresh, conf)
        return result

    def img_preprocessing(self, orig_img, pixel_means, 
            max_size=1000, scale=600):
        img = orig_img.astype(np.float32, copy=True)
        img -= pixel_means
        im_size_min = np.min(img.shape[0:2])
        im_size_max = np.max(img.shape[0:2])
        im_scale = float(scale) / float(im_size_min)
        if np.round(im_scale * im_size_max) > max_size:
            im_scale = float(max_size) / float(im_size_max)
        img = cv.resize(img, None, None, fx=im_scale, fy=im_scale,
                        interpolation=cv.INTER_LINEAR)

        return img.transpose([2, 0, 1]).astype(np.float32), im_scale

    def draw_result(self, out, im_scale, clss, bbox, nms_thresh, conf):
        CV_AA = 16
        for cls_id in range(1, 21):
            _cls = clss[:, cls_id][:, np.newaxis]
            _bbx = bbox[:, cls_id * 4: (cls_id + 1) * 4]
            dets = np.hstack((_bbx, _cls))
            keep = nms(dets, nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= conf)[0]
            for i in inds:
                x1, y1, x2, y2 = map(int, dets[i, :4])
                cv.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2, CV_AA)
                ret, baseline = cv.getTextSize(
                    CLASSES[cls_id], cv.FONT_HERSHEY_SIMPLEX, 0.8, 1)
                cv.rectangle(out, (x1, y2 - ret[1] - baseline),
                             (x1 + ret[0], y2), (0, 0, 255), -1)
                cv.putText(out, CLASSES[cls_id], (x1, y2 - baseline),
                           cv.FONT_HERSHEY_SIMPLEX, 
                           0.8, (255, 255, 255), 1, CV_AA)
        return out

    def get_dictionary(self, out, im_scale, clss, bbox, nms_thresh, conf):
        dictionary = {}
        width = out.shape[1]
        height = out.shape[0]
        for cls_id in range(1, 21):
            _cls = clss[:, cls_id][:, np.newaxis]
            _bbx = bbox[:, cls_id * 4: (cls_id + 1) * 4]
            dets = np.hstack((_bbx, _cls))
            keep = nms(dets, nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= conf)[0]
            for i in inds:
                x1, y1, x2, y2 = map(int, dets[i, :4])
                if x2 > width:
                    x2 = width
                if y2 > height:
                    y2 = height
                name = CLASSES[cls_id] + "_" + str(i)
                dictionary[name] = (x1, y1, x2, y2)
        return dictionary

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=str, default='sample.jpg')
    parser.add_argument('--draw', action="store_true")
    parser.add_argument('--gpu', type=int, default=-1)
    args = parser.parse_args()
    
    net = FRCNN(args.gpu)
    net.set_model("VGG16_faster_rcnn_final.model")

    image = cv.imread(args.input)
    out = net.detect_object(image, draw = args.draw)
    if args.draw:
        cv.imshow("RESULT", out)
        cv.waitKey(0)
    else:
        print out
    return 0

if __name__ == "__main__":
    import argparse

    main()
