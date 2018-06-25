#!/bin/sh

#
# Usage: sh install-fedora.sh
#

echo "UsTK installation Script for Fedora"
echo "Installing dependencies..."

sudo yum install make automake gcc gcc-c++ kernel-devel git
sudo yum install cmake
sudo yum install opencv-devel
sudo yum install libX11-devel
sudo yum install libxml2-devel
sudo yum install vtk-devel vtk-qt 
sudo yum install fftw
sudo yum install armadillo-devel

if [ ! -v USTK_WS ] 
then
    echo "USTK_WS is unset, please set it to point on a directory to put ViSP/UsTK sources and binaries. You can use the following command:"
    echo "export USTK_WS=~/Documents/ustk-ws"
elif [ -z "$USTK_WS" ]
then
    echo "USTK_WS is unset, please set it to point on a directory to put ViSP/UsTK sources and binaries. You can use the following command:"
    echo "export USTK_WS=~/Documents/ustk-ws"
else
    echo "Getting ViSP source..."
    if [ ! -d "$USTK_WS/visp" ]; then
       git clone https://github.com/lagadic/visp $USTK_WS/visp
    else
       cd $USTK_WS/visp
       git pull origin master
    fi

    if [ ! -d "$USTK_WS/ustk-build" ]; then
       echo "Creating Build directory: $USTK_WS/ustk-build"
       mkdir $USTK_WS/ustk-build
    fi
	
    cd $USTK_WS/ustk-build

    echo "Configuring project with CMake..."
    cmake $USTK_WS/visp -DVISP_CONTRIB_MODULES_PATH=$USTK_WS/ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF
	 
    echo "Compiling project"
    make -j4

    echo "Importing ustk-dataset"
    if [ ! -d "$USTK_WS/ustk-dataset" ]; then
       git clone https://github.com/lagadic/ustk-dataset $USTK_WS/ustk-dataset
    else
       cd $USTK_WS/ustk-dataset
       git pull origin master
    fi
    export USTK_DATASET_PATH=$USTK_WS/ustk-dataset
fi

