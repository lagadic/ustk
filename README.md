### Ultrasound Toolkit - UsTk

Copyright (C) 2016 by Inria.

#### Dependencies
This UsTk project needs [ViSP](https://visp.inria.fr) as third-party.
For compilation, UsTk is considered as a set of ViSP external modules. So ViSP and UsTk have to be compiled together. 

#### How to build UsTK libraries
- Create a workspace folder en enter in this folder

		$ mkdir <workspace>; cd <workspace>
- Downloaded ViSP and UsTk sources codes

		$ git clone https://github.com/lagadic/visp
		$ git clone https://github.com/lagadic/ustk
		
- Make a build directory (where UsTk and ViSP will be compiled)
 		
		$ mkdir ustk-build; cd ustk-build
		
- Configure the build setting UsTk as an external ViSP module

		$ cmake ../visp -DVISP_CONTRIB_MODULES_PATH=../ustk
		
- Note that with the previous command, all ViSP modules will be build besides UsTk. Since ViSP modules related to AR, detection, computer vision or tracking are not used by UsTk, their build could be turned off in order to speed up UsTk build. This could be achieved using:
 
		$ cmake ../visp -DVISP_CONTRIB_MODULES_PATH=../ustk -DBUILD_MODULE_visp_ar=OFF -DBUILD_MODULE_visp_blob=OFF -DBUILD_MODULE_visp_detection=OFF -DBUILD_MODULE_visp_klt=OFF -DBUILD_MODULE_visp_mbt=OFF -DBUILD_MODULE_visp_me=OFF -DBUILD_MODULE_visp_tt=OFF -DBUILD_MODULE_visp_tt_mi=OFF -DBUILD_MODULE_visp_vision=OFF -DBUILD_MODULE_visp_visual_features=OFF -DBUILD_MODULE_visp_vs=OFF   
		
- Now on unix-like OS build ViSP and UsTk using

		$ make -j

#### How to build UsTK documentation

UsTk documentation is part of ViSP documentation. To build UsTk doxygen documentation, run

	$ make -j visp_doc

