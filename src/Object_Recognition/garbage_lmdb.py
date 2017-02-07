import caffe
from caffe.proto import caffe_pb2
import cv2
from glob import glob
import lmdb
import numpy as np
import os
from random import shuffle

DIR = "/home/jbweibel/code/vvv17-demo-team-green/"
DATASET_DIR = DIR + "DATA/"

IMAGE_WIDTH = 227
IMAGE_HEIGHT = 227

LABELS = DIR + "labels_washington.txt"

class_to_label = {}
with open(LABELS, "r") as fp:
    for idx, line in enumerate(fp):
        class_to_label[line[:-1]] = idx

label_to_class = {}
for k,v in class_to_label.iteritems():
    label_to_class[v] = k

def preprocess(img, img_width=IMAGE_WIDTH, img_height=IMAGE_HEIGHT):
    #Histogram Equalization
    img[:,:,0] = cv2.equalizeHist(img[:, :, 0])
    img[:,:,1] = cv2.equalizeHist(img[:, :, 1])
    img[:,:,2] = cv2.equalizeHist(img[:, :, 2])

    #Image Resizing
    img = cv2.resize(img, (img_width, img_height), interpolation = cv2.INTER_CUBIC)

    return img

def make_datum(img, label):
    #image is numpy.ndarray format. BGR instead of RGB
    return caffe_pb2.Datum(
        channels=3,
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        label=label,
        data=np.rollaxis(img,2).tostring())

def create_lmdb(data, lmdb_name):
    in_db = lmdb.open(lmdb_name, map_size=int(1e12))
    with in_db.begin(write=True) as in_txn:
        for in_idx, img_path in enumerate(data):
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            img = preprocess(img)
            label = class_to_label[img_path.split("/")[-2]]

            datum = make_datum(img, label)
            in_txn.put('{:0>5d}'.format(in_idx), datum.SerializeToString())

            if in_idx % 100 == 0:
                print '{:0>5d} / {:d}'.format(in_idx, len(data)) + ':' + img_path


def create_train_val_lmdb(depth=True):
    classes = os.listdir(DATASET_DIR)
    for classname in classes:
        data += glob(DATASET_DIR + classname + "/*")

    shuffle(data)

    split = len(data) / 6
    val_data = filenames[:split]
    train_data = filenames[split:]

    train_lmdb = DIR + 'input/train_lmdb'
    validation_lmdb = DIR + 'input/validation_lmdb'

    os.system('rm -rf  ' + train_lmdb)
    os.system('rm -rf  ' + validation_lmdb)

    print 'Creating train_lmdb'
    create_lmdb(train_data, train_lmdb)

    print 'Creating val_lmdb'
    create_lmdb(val_data, validation_lmdb)

