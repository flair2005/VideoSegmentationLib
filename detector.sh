#!/bin/bash

scales=3
starting_scale=2
prop_scale=0
gpu=0 
#svm=/home/martin/bagfiles/SLC/slc_svm_model_friday.xml
svm=/home/martin/workspace/VideoSegmentationLib/slc/slc_svm_model_bbox2.xml
#svm=/home/martin/workspace/VideoSegmentationLib/slc/slc_svm_model_friday.xml

imgDir=/home/martin/bagfiles/pickup_table_1/img/
depthDir=/home/martin/bagfiles/pickup_table_1/depth/
output=/home/martin/tmp/tmp

	    
ls -1v $imgDir/*.png > imgs.txt
ls -1v $depthDir/*.png > depths.txt
build/test_detector -i imgs.txt -c depths.txt  -v $svm -o $output -s $scales -r $starting_scale  -p $prop_scale

