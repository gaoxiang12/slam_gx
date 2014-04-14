#!/bin/sh
# build the template project

clear
mkdir build
cd build
cmake DCMAKE_BUILD_TYPE = Debug ..
make 2> error.log
cat error.log

cd ..
rm log/*
rm debug.txt
rm g2o/*
rm ransac_keypoint.txt
rm keypoint.txt
rm kp.txt
rm p3d.txt
rm keyframe/*
rm final.pcd
