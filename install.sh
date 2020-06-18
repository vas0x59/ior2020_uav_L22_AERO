#!/bin/bash
FILE_NAME="ior2020_uav_L22_AERO-master"
ZIP=".zip"
cd ~
mkdir -p ~/L22_DayTwo/src
cd ~/L22_DayTwo/
mv ~/$FILE_NAME$ZIP src
cd src
unzip -o $FILE_NAME$ZIP
cd ../
catkin_make
source devel/setup.bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/src/$FILE_NAME