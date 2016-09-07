Ultrasound Toolkit - UsTk-1.0.0

Copyright (C) 2014 by Inria.

This UsTk project needs ViSP source code (https://github.com/lagadic/visp).
For compilation, UsTk is considered as a set of ViSP modules. So ViSP and UsTk have to be compiled together. 

To compile :
- make sure you have downloaded ViSP and UsTk sources codes.
- create a binary directory
- from the binary directory, run : cmake 'path_to_visp' -DVISP_CONTRIB_MODULES_PATH='path_to_UsTk'
- then CMake will create a project you can compile

Check https://visp.inria.fr/ for documentation.


This project is using the CMake build system.

CMake is a complete stand-alone platform-independant build-system 
replacing autotools (autoconf/autoheader/automake/libtools) completely. 
It depends just on installed cmake (tested with cmake cvs version). It
needs a cmake 2.6.x or more recent version of cmake.
See http://www.cmake.org for details.

USAGE:
=====

1. On Unix platforms:

cd <ustk build dir>
ccmake <ustk source dir>
make
make install

2. On Windows, use the CMake GUI.

----------------
Pierre Chatelain
Marc Pouliquen

