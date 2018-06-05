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
 * Author:
 * Marc Pouliquen
 *
 *****************************************************************************/
#include <iostream>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>
#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

int main()
{
  std::string filename, filenameConfidence;

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (!env_ipath.empty()) {
      filename = env_ipath + "/pre-scan/2D_xml/prescan2d.xml";
      filenameConfidence = env_ipath + "/pre-scan/2D_xml/confidenceResult/confidencePreScan.xml";
    } else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImagePreScan2D<unsigned char> image, confidenceTest, confidenceGroundTruth;
  usImageIo::read(image, filename);
  usImageIo::read(confidenceGroundTruth, filenameConfidence);

  // run confidence map
  usScanlineConfidence2D confidenceProcess;
  confidenceProcess.run(confidenceTest, image);

  std::cout << "Output image : \n";
  std::cout << confidenceTest;
  std::cout << "\nGround truth image : \n";
  std::cout << confidenceGroundTruth;

  if (confidenceTest == confidenceGroundTruth) {
    std::cout << "Test passed\n";
    return 0;
  }

  std::cout << "Test failed\n";
  return 1;
}
