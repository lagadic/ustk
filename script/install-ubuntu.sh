#!/bin/sh

#
# Usage: sh install-ubuntu.sh
#
script_path=`dirname $0`
ustk_root=`(cd $script_path/..; pwd)`
src_root=`(cd $ustk_root/..; pwd)`

echo UsTK installation Script
echo Installing dependencies...

sudo apt-get install build-essential
sudo apt-get install cmake-curses-gui
sudo apt-get install libopencv-dev
sudo apt-get install libx11-dev
sudo apt-get install libxml2-dev
sudo apt-get install libvtk6-qt-dev
sudo apt-get install libfftw3-dev

echo Getting ViSP source...
git clone https://github.com/lagadic/visp $src_root/visp

echo Creating Build directory: $src_root/ustk_build
mkdir $src_root/ustk_build
cd $src_root/ustk_build

echo configuring project...
cmake ../visp -DVISP_CONTRIB_MODULES_PATH=../ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF
 
echo Compiling project
make -j4

echo importing ustk-dataset
git clone https://github.com/lagadic/ustk-dataset $src_root/ustk-dataset
export USTK_DATASET_PATH=$src_root/ustk-dataset

