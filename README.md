### Ultrasound Tool Kit - UsTK

Copyright (C) 2016 by Inria.

[![Build Status](https://travis-ci.org/lagadic/ustk.png)](https://travis-ci.org/lagadic/ustk)

#### Dependencies

This UsTK project needs [ViSP](https://visp.inria.fr) and [VTK](http://www.vtk.org/) with Qt4 or Qt5 support as third-party.

- ViSP: UsTK is considered as a set of ViSP external modules. Thus ViSP and UsTK have to be compiled together. It means that you should get ViSP source code as explained in the next section.

- VTK: Should be installed from pre-build packages


  - How to install VTK OS X

			$ brew update
			$ brew install vtk —-with-qt5
		
  - How to install VTK on Ubuntu 16.04 LTS
  
			$ sudo apt-get install libvtk6-qt-dev
			
  - How to install VTK on Fedora 23

			$ sudo yum install vtk-devel vtk-qt
 		 

#### How to build UsTK libraries

- Create a workspace folder and enter in this folder

		$ mkdir <workspace>; cd <workspace>

- Downloaded ViSP and UsTK sources codes

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

#### How to build UsTK documentation
 
There are two ways to build UsTK documentation.

- If UsTK was build as an external contrib module (see previous section), UsTK documentation is part of ViSP documentation. To build UsTK documentation that will be part of ViSP documentation, run

		$ cd visp-ustk-build
		$ make -j4 visp_doc

- It is also possible to build UsTK documentation as a stand alone documentation, meaning without embedding ViSP documentation. To this end create a new build folder `ustk-build-doc` to host the stand alone documentation and make `ustk_doc` target:

		$ cd <workspace>
		$ mkdir ustk-build-doc
		$ cd ustk-build-doc
		$ cmake ../ustk
		$ make -j4 ustk_doc

#### How to use UsTK data set

Some examples or tutorials are working with ultrasound medical images. We propose a data set that contains 2D or 3D ultrasound data in <https://gitlab.inria.fr/lagadic/ustk-dataset>. To use this data set you may set the USTK_DATASET_PATH environment variable like:

	$ cd <workspace>
	$ git clone https://gitlab.inria.fr/lagadic/ustk-dataset
	$ export USTK_DATASET_PATH=<workspace>/ustk-dataset




