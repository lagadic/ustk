#!/bin/sh

#
# Usage: sh install-ubuntu.sh
#
echo "UsTK installation Script"
echo "Installing dependencies..."

# Check ubuntu version
UBUNTU_VERSION=$(grep 'VERSION_ID' /etc/os-release | grep -o -E '[0-9]+.[0-9]+')
# Compare to 18.04, to know if libarmadillo8 package is available
UBUNTU_VERSION_18_04='18.04'
LATEST_VERSION="`echo -e "$UBUNTU_VERSION\n$UBUNTU_VERSION_18_04" | sort -V | head -n1`"

if [ "$LATEST_VERSION" = "$UBUNTU_VERSION_18_04" ]; then
    echo "ustk_elastography module requires armadillo 6.7 or newer (ubuntu package includes too old version of armadillo on ubuntu versions prior to 18.04), so ustk_elastography will not be built."
else
    sudo apt-get install libarmadillo8
fi

sudo apt-get install build-essential
sudo apt-get install cmake-curses-gui
sudo apt-get install libopencv-dev
sudo apt-get install libx11-dev
sudo apt-get install libxml2-dev
sudo apt-get install libvtk6-qt-dev
sudo apt-get install libfftw3-dev
sudo apt-get install libarmadillo-dev

if [ ! -v USTK_WS ] 
then
    echo "USTK_WS is unset, please set it to point on a directory to put ViSP/UsTK sources and binaries"
elif [ -z "$USTK_WS" ]
then
    echo "USTK_WS is empty, please set it to point on a directory to put ViSP/UsTK sources and binaries"
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

    if [ "$LATEST_VERSION" = "$UBUNTU_VERSION_18_04" ]; then  
        cmake $USTK_WS/visp -DVISP_CONTRIB_MODULES_PATH=$USTK_WS/ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF -DBUILD_MODULE_visp_ustk_elastography=OFF
    else    
        cmake $USTK_WS/visp -DVISP_CONTRIB_MODULES_PATH=$USTK_WS/ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF
	 
    fi

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

