# Ultrasound Tool Kit - UsTK

Copyright (C) 2016-2017 by Inria.

Platform | Build Status |
-------- | ------------ |
Linux and OS X | [![Build Status](https://travis-ci.org/lagadic/ustk.png)](https://travis-ci.org/lagadic/ustk) |
Windows | [![Build status](https://ci.appveyor.com/api/projects/status/25t7wcl7akdw3nmw/branch/master?svg=true)](https://ci.appveyor.com/project/fspindle/ustk/branch/master) |


## Dependencies

This UsTK project requires the following libraries :

- [ViSP](https://visp.inria.fr) : UsTK is considered as a set of ViSP external modules. Thus ViSP and UsTK have to be compiled together from source. It means that you should get ViSP source code as explained in the next section.

- [XML2](http://xmlsoft.org/): XML2 library is used by UsTK for I/O (ultrasound image or sequence of images settings). This 3rd party could be installed from existing packages.

	- How to install XML2 on OS X

			$ brew update
			$ brew install libxml2

	- How to install XML2 on Ubuntu 16.04 LTS

			$ sudo apt-get install libxml2-dev

	- How to install XML2 on Fedora 23

			$ sudo yum install libxml2-devel
      
	- How to install XML2 on Windows

		On Windows, you should install XML2 from source as explained [here](https://visp.inria.fr/3rd_xml2/).

- [VTK](http://www.vtk.org/) with Qt4 or Qt5 support: VTK library is used by UsTK for visualization (GUI). This 3rd party could be installed from existing pre-build packages.

	- How to install VTK on OS X

			$ brew update
			$ brew install vtk —-with-qt

	- How to install VTK on Ubuntu 16.04 LTS

			$ sudo apt-get install libvtk6-qt-dev

	- How to install VTK on Fedora 23

			$ sudo yum install vtk-devel vtk-qt
      
	- How to install VTK on Windows
  
		- On Windows, you should first get and install Qt5 open-source package from [here](https://info.qt.io/download-qt-for-application-development). Next steps were tested with Qt 5.9.1 and Microsoft Visual 2017.
		  In the installer select the component corresponding to your IDE: `Qt > Qt 5.9.1 > msvc2017 64-bit`. The installation should bring Qt5 in a folder named `C:\my_path\Qt\5.9.1\msvc2017_64`.
			Now to help CMake detect Qt5 you should set `Qt5_DIR` environment var:

				setx Qt5_DIR "C:\Users\fspindle\soft\Qt\5.9.1\msvc2017_64\lib\cmake"
		
		- Then you should install VTK from source as explained [here](http://www.vtk.org/Wiki/VTK/Building/Windows). 
			Additional steps should be done to configure VTK with Qt5:
		  
			- You should enable Qt support enabling `VTK_Group_Qt` var in CMakeGUI. 
			- Select configure in CMakeGUI and select `VTK_QT_VERSION=5`. 
			- Set `CMAKE_INSTALL_PREFIX` to `C:\MyProjects\VTK-bin\install`.
		
		- In order to be detected by CMake, you should than add `VTK_DIR` environment variable which points to the installation folder
		
				setx VTK_DIR "C:\my_path\VTK-bin\install\lib\cmake\vtk-8.0"
		
		- In order that installed VTK libraries are found during execution of UsTK binaries, you should also add the folder that contains these libraies to the `PATH` environment variable.
		This folder is the one corresponding to the one that you have set in `CMAKE_INSTALL_PREFIX` suffixed by `\bin`. For example if `CMAKE_INSTALL_PREFIX` is set to `C:\my_path\VTK-bin\install` you can run:
		
				setx PATH "%PATH%;C:\my_path\VTK-bin\install\bin"
 		 
- [FFTW](http://www.fftw.org/): FFTW library is used by RF to pre-scan converters. This 3rd party could be installed from existing pre-build packages. 

	- How to install FFTW OS X

			$ brew update
			$ brew install fftw
		
	- How to install FFTW on Ubuntu 16.04 LTS
  
			$ sudo apt-get install libfftw3-dev
			
	- How to install FFTW on Fedora 23

			$ sudo yum install fftw
		
	- How to use FFTW on Windows 
  
		Windows installation instructions can be found [here](http://www.fftw.org/install/windows.html). 
		You will have to download the binaries, and create the .lib files from the .def files. 
		To detect automatically fftw with CMake you have to set the environment variable : FFTW_HOME, pointing on your fftw binary directory. You can use the following command :
	
			$ setx FFTW_HOME C:/path/to/fftw
	
		Then don't forget to add the folder to your PATH.
	  
- [armadillo](http://arma.sourceforge.net/): armadillo library is used for elastography. This 3rd party could be installed from existing pre-build packages.

	- How to install armadillo OS X

			$ brew update
			$ brew install armadillo

	- How to install armadillo on Ubuntu

			$ sudo apt-get install libarmadillo-dev

	- How to install armadillo on Fedora

			$ sudo yum install armadillo-devel

## How to build UsTK binaries

- Install XML2, VTK, FFTW, armadillo dependencies as explained in the previous section.

- Create a workspace folder and enter in this folder

		$ mkdir <workspace>; cd <workspace>

- Download ViSP and UsTK sources codes

		$ git clone https://github.com/lagadic/visp
		$ git clone https://github.com/lagadic/ustk

- Make a build directory (where UsTK and ViSP will be compiled together)

		$ mkdir visp-ustk-build; cd visp-ustk-build

- Configure the build setting UsTK as a ViSP external contrib module

		$ cmake ../visp -DVISP_CONTRIB_MODULES_PATH=../ustk

- Note that with the previous command, all ViSP modules will be build besides UsTK. Since ViSP modules related to AR, detection, computer vision or tracking are not used by UsTK, their build could be turned off in order to speed up UsTK build. This could be achieved using:

		$ cmake ../visp -DVISP_CONTRIB_MODULES_PATH=../ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF

- Now on unix-like OS build ViSP and UsTK using:

		$ make -j4

### How to build UsTK documentation

To build UsTK documentation as a stand alone documentation (i.e.. without all ViSP classes prefixed by "vp"):

	$ make -j4 ustk_doc


### How to use UsTK data set

Some examples or tutorials are working with ultrasound medical images. We propose a data set that contains 2D or 3D ultrasound data in <https://github.com/lagadic/ustk-dataset>. To use this data set you may set the USTK_DATASET_PATH environment variable like:

	$ cd <workspace>
	$ git clone https://github.com/lagadic/ustk-dataset
	$ export USTK_DATASET_PATH=<workspace>/ustk-dataset


## Known issues

### No rule to make target '/usr/lib/x86_64-linux-gnu/libproj.so'

This issue may appear on Ubuntu 16.04 LTS

	$ make
	make[3]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libproj.so', needed by 'lib/libvisp_ustk_gui.so.3.0.2'.  Stop.
	CMakeFiles/Makefile2:6995: recipe for target 'modules/ustk_gui/CMakeFiles/visp_ustk_gui.dir/all' failed

This issue is related to vtk installation where libproj.so is a dependency that is not installed with vtk.

	$ grep libproj /usr/lib/cmake/vtk-6.2/VTKTargets.cmake
	INTERFACE_LINK_LIBRARIES "vtkIOXML;vtkInfovisLayout;vtkInteractionStyle;vtkInteractionWidgets;vtkRenderingCore;vtkViewsCore;/usr/lib/x86_64-linux-gnu/libproj.so"

The fix consists in installing libproj-dev package:

	$ sudo apt-get install libproj-dev
	
