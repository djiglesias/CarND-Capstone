#!/usr/bin/env python

# Usage
# Credit: https://github.com/ksakmann/team-robo4

import os
import sys
from functools import partial

chunksize = 1024
maxchunks = 40000

def splitfile(filename, directory, chunksize=chunksize, maxchunks=maxchunks):
    if not os.path.exists(directory):
        os.mkdir(directory)
    else:
        for fname in os.listdir(directory):
            os.remove(os.path.join(directory, fname))
    chunknum = 0
    with open(filename, 'rb') as infile:
        for chunk in iter(partial(infile.read, chunksize*maxchunks), ''):
            ofilename = os.path.join(directory, ('chunk%04d'%(chunknum)))
            outfile = open(ofilename, 'wb')
            outfile.write(chunk)
            outfile.close()
            chunknum += 1
            print("chunky....chunk",chunknum,len(chunk))
            if len(chunk) == 0:
                exit()

splitfile('../light_classification/models/rfcn_resnet101_coco_2018_01_28/frozen_inference_graph.pb',
'../light_classification/model_chunks/rfcn_resnet101_coco_2018_01_28')
