/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file us.cpp
 * @brief us namespace.
 */


#include <visp3/ustk_core/us.h>

namespace us {

  /*!
   Get UsTK data set path. UsTK data set can be downloaded from https://gitlab.inria.fr/lagadic/ustk-dataset.

   This function returns the path to the folder that contains the data.
   - It checks first if the data set is installed in \c /usr/share/ustk-dataset. In that case returns then \c "/usr/share/ustk-dataset".
   - Then it checks if USTK_DATASET_PATH environment variable that gives the location of the data is set. In that
     case returns the content of this environment var.

   If the path is not found, returns an empty string.
   */
  std::string getDataSetPath()
  {
    std::string data_path;
    std::string file_to_test("pre-scan/3D_mhd/volume.mhd");
    std::string filename;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    // Test if ustk-data package is installed (Ubuntu and Debian)
    data_path = "/usr/share/ustk-dataset";
    filename = data_path + "/" + file_to_test;
    if (vpIoTools::checkFilename(filename))
      return data_path;
#endif
    // Test if USTK_DATASET_PATH env var is set
    try {
      data_path = vpIoTools::getenv("USTK_DATASET_PATH");
      filename = data_path + "/" + file_to_test;
      if (vpIoTools::checkFilename(filename))
        return data_path;
    }
    catch(...) {
    }
    data_path = "";
    return data_path;
  }
};

