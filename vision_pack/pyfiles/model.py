import tensorflow as tf
import numpy as np
import sys
from network import *

class Model:
    @staticmethod
    def alexnet(_X, _dropout, _output_dim=20):
        # TODO weight decay loss tern
        # Layer 1 (conv-relu-pool-lrn)
        conv1 = conv(_X, 11, 11, 96, 4, 4, padding='VALID', name='conv1')
        conv1 = max_pool(conv1, 3, 3, 2, 2, padding='VALID', name='pool1')
        norm1 = lrn(conv1, 2, 2e-05, 0.75, name='norm1')
        # Layer 2 (conv-relu-pool-lrn)
        conv2 = conv(norm1, 5, 5, 256, 1, 1, group=2, name='conv2')
        conv2 = max_pool(conv2, 3, 3, 2, 2, padding='VALID', name='pool2')
        norm2 = lrn(conv2, 2, 2e-05, 0.75, name='norm2')
        # Layer 3 (conv-relu)
        conv3 = conv(norm2, 3, 3, 384, 1, 1, name='conv3')
        # Layer 4 (conv-relu)
        conv4 = conv(conv3, 3, 3, 384, 1, 1, group=2, name='conv4')
        # Layer 5 (conv-relu-pool)
        conv5 = conv(conv4, 3, 3, 256, 1, 1, group=2, name='conv5')
        pool5 = max_pool(conv5, 3, 3, 2, 2, padding='VALID', name='pool5')
        # Layer 6 (fc-relu-drop)
        fc6 = tf.reshape(pool5, [-1, 6*6*256])
        fc6 = fc(fc6, 6*6*256, 4096, name='fc6')
        fc6 = dropout(fc6, _dropout)
        # Layer 7 (fc-relu-drop)
        fc7 = fc(fc6, 4096, 4096, name='fc7')
        fc7 = dropout(fc7, _dropout)
        # Layer 8 (fc-prob)
        fc8 = fc(fc7, 4096, _output_dim, relu=False, name='fc8')
        return fc8

    @staticmethod
    def alexnet_fe(_X, _dropout, _output_dim=1000):
        # TODO weight decay loss tern
        # Layer 1 (conv-relu-pool-lrn)
        conv1 = conv(_X, 11, 11, 96, 4, 4, padding='VALID', name='conv1')
        conv1 = max_pool(conv1, 3, 3, 2, 2, padding='VALID', name='pool1')
        norm1 = lrn(conv1, 2, 2e-05, 0.75, name='norm1')
        # Layer 2 (conv-relu-pool-lrn)
        conv2 = conv(norm1, 5, 5, 256, 1, 1, group=2, name='conv2')
        conv2 = max_pool(conv2, 3, 3, 2, 2, padding='VALID', name='pool2')
        norm2 = lrn(conv2, 2, 2e-05, 0.75, name='norm2')
        # Layer 3 (conv-relu)
        conv3 = conv(norm2, 3, 3, 384, 1, 1, name='conv3')
        # Layer 4 (conv-relu)
        conv4 = conv(conv3, 3, 3, 384, 1, 1, group=2, name='conv4')
        # Layer 5 (conv-relu-pool)
        conv5 = conv(conv4, 3, 3, 256, 1, 1, group=2, name='conv5')
        pool5 = max_pool(conv5, 3, 3, 2, 2, padding='VALID', name='pool5')
        # Layer 6 (fc-relu-drop)
        fc6 = tf.reshape(pool5, [-1, 6*6*256])
        fc6, fc6wi = fe(fc6, 6*6*256, 4096, name='fc6')
        fc6 = dropout(fc6, _dropout)
        # Layer 7 (fc-relu-drop)
        fc7 = fc(fc6, 4096, 4096, name='fc7')
        fc7 = dropout(fc7, _dropout)
        # Layer 8 (fc-prob)
        fc8 = fc(fc7, 4096, _output_dim, relu=False, name='fc8')
        return fc8, fc6wi

    @staticmethod
    def VGG_NET(_X, _dropout, _output_dim=2622):
        # TODO weight decay loss tern
        # Layer 1 (conv-conv-pool)
        conv1_1 = conv(_X, 3, 3, 64, 1, 1, name='vgg_conv1_1')
        conv1_2 = conv(conv1_1, 3, 3, 64, 1, 1, name='vgg_conv1_2')
        pool1 = max_pool(conv1_2, 2, 2, 2, 2, name='vgg_pool1')
        # Layer 2 (conv-conv-pool)
        conv2_1 = conv(pool1, 3, 3, 128, 1, 1, name='vgg_conv2_1')
        conv2_2 = conv(conv2_1, 3, 3, 128, 1, 1, name='vgg_conv2_2')
        pool2 = max_pool(conv2_2, 2, 2, 2, 2, name='vgg_pool2')
        # Layer 3 (conv-conv-conv-pool)
        conv3_1 = conv(pool2, 3, 3, 256, 1, 1, name='vgg_conv3_1')
        conv3_2 = conv(conv3_1, 3, 3, 256, 1, 1, name='vgg_conv3_2')
        conv3_3 = conv(conv3_2, 3, 3, 256, 1, 1, name='vgg_conv3_3')
        pool3 = max_pool(conv3_3, 2, 2, 2, 2, name='vgg_pool3')
        # Layer 4 (conv-conv-conv-pool)
        conv4_1 = conv(pool3, 3, 3, 512, 1, 1, name='vgg_conv4_1')
        conv4_2 = conv(conv4_1, 3, 3, 512, 1, 1, name='vgg_conv4_2')
        conv4_3 = conv(conv4_2, 3, 3, 512, 1, 1, name='vgg_conv4_3')
        pool4 = max_pool(conv4_3, 2, 2, 2, 2, name='vgg_pool4')
        # Layer 5 (conv-conv-conv-pool)
        conv5_1 = conv(pool4, 3, 3, 512, 1, 1, name='vgg_conv5_1')
        conv5_2 = conv(conv5_1, 3, 3, 512, 1, 1, name='vgg_conv5_2')
        conv5_3 = conv(conv5_2, 3, 3, 512, 1, 1, name='vgg_conv5_3')
        pool5 = max_pool(conv5_3, 2, 2, 2, 2, name='vgg_pool5')
        # Layer 6 (fc-relu-drop)
        fc6 = tf.reshape(pool5, [-1, 7*7*512])
        fc6, fc6wi = fe(fc6, 7*7*512, 4096, name='vgg_fc6')
        fc6 = dropout(fc6, _dropout)
        # Layer 7 (fc-relu-drop)
        fc7 = fc(fc6, 4096, 4096, name='vgg_fc7')
        fc7 = dropout(fc7, _dropout)
        # Layer 8 (fc-prob)
        fc8 = fc(fc7, 4096, _output_dim, relu=False, name='vgg_fc8')
        return fc8, fc6wi


    @staticmethod
    def AGE_NET(_X, _dropout):
        # TODO weight decay loss tern
        # Layer 1 (conv-pool-lrn)
        conv1 = conv(_X, 7, 7, 96, 4, 4, name='age_conv1', padding='VALID')
        pool1 = max_pool(conv1, 3, 3, 2, 2, name='age_pool1')
        lrn1  = lrn(pool1, 2, 2e-05, 0.75, name='age_norm1')
        # Layer 2 (conv-pool-lrn)
        conv2 = conv(lrn1, 5, 5, 256, 1, 1, name='age_conv2')
        pool2 = max_pool(conv2, 3, 3, 2, 2, name='age_pool2')
        lrn2  = lrn(pool2, 2, 2e-05, 0.75, name='age_norm2')
        # Layer 3 (conv-pool)
        conv3 = conv(lrn2, 3, 3, 384, 1, 1, name='age_conv3')
        pool5 = max_pool(conv3, 3, 3, 2, 2, name='age_pool5')
        # Layer 4 (fc-relu-drop)
        fc6 = tf.reshape(pool5, [-1, 7*7*384])
        fc6 = fc(fc6, 7*7*384, 512, name='age_fc6')
        fc6 = dropout(fc6, _dropout)
        # Layer 5 (fc-relu-drop)
        fc7 = fc(fc6, 512, 512, name='age_fc7')
        fc7 = dropout(fc7, _dropout)
        # Layer 6 (fc)
        fc8 = fc(fc7, 512, 8, relu=False, name='age_fc8')
        return fc8

    @staticmethod
    def GENDER_NET(_X, _dropout):
        # TODO weight decay loss tern
        # Layer 1 (conv-pool-lrn)
        conv1 = conv(_X, 7, 7, 96, 4, 4, name='gender_conv1', padding='VALID')
        pool1 = max_pool(conv1, 3, 3, 2, 2, name='gender_pool1')
        lrn1  = lrn(pool1, 2, 2e-05, 0.75, name='gender_norm1')
        # Layer 2 (conv-pool-lrn)
        conv2 = conv(lrn1, 5, 5, 256, 1, 1, name='gender_conv2')
        pool2 = max_pool(conv2, 3, 3, 2, 2, name='gender_pool2')
        lrn2  = lrn(pool2, 2, 2e-05, 0.75, name='gender_norm2')
        # Layer 3 (conv-pool)
        conv3 = conv(lrn2, 3, 3, 384, 1, 1, name='gender_conv3')
        pool5 = max_pool(conv3, 3, 3, 2, 2, name='gender_pool5')
        # Layer 4 (fc-relu-drop)
        fc6 = tf.reshape(pool5, [-1, 7*7*384])
        fc6 = fc(fc6, 7*7*384, 512, name='gender_fc6')
        fc6 = dropout(fc6, _dropout)
        # Layer 5 (fc-relu-drop)
        fc7 = fc(fc6, 512, 512, name='gender_fc7')
        fc7 = dropout(fc7, _dropout)
        # Layer 6 (fc)
        fc8 = fc(fc7, 512, 2, relu=False, name='gender_fc8')
        return fc8
