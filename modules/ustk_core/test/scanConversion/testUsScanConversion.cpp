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

#include <visp3/core/vpConfig.h>

#include <iostream>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>

#include <visp3/ustk_core/us.h>
#include <visp3/ustk_core/usConfig.h>
#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usPostScanToPreScan2DConverter.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>

#include <string>
#include <vector>

/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */
#if defined(VISP_HAVE_XML2)
int main(int argc, const char **argv)
{
  (void)argc;
  (void)argv;

  if (us::getDataSetPath().empty())
    throw(vpException(vpException::fatalError, "Could not find ustk-dataset"));

  bool testFailed = true;

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "  testUsScanConversion2D.cpp" << std::endl << std::endl;
  std::cout << "  The test converts a pre-scan image to post-scan, and converts it back to pre-scan."
               "  Then it checks if the difference between the 2 images, using sum of square differences."
            << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  // pre-scan reading
  usImagePreScan2D<unsigned char> prescanReference;
  usImagePostScan2D<unsigned char> postscan;
  usImagePreScan2D<unsigned char> prescanBack;

  std::string preScanPath = us::getDataSetPath() + "/pre-scan/2D_xml/prescan2d.xml";

  usImageIo::read(prescanReference, preScanPath);

  usPreScanToPostScan2DConverter scanConverter;
  scanConverter.convert(prescanReference, postscan, 0.0005, 0.0005);

  usPostScanToPreScan2DConverter backScanConverter;
  backScanConverter.convert(postscan, prescanBack, 480);

  // compute the sum of square difference of the reference and the result
  int inc = 0;
  int pxSkipped = 0;
  for (unsigned int i = 0; i < prescanBack.getHeight(); i++) {
    for (unsigned int j = 0; j < prescanBack.getWidth(); j++) {
      if (i > 3 && j > 3) { // we skip top & left borders, they have huge differencies compared to the rest of the image
        inc += (prescanBack[i][j] - prescanReference[i][j]) * (prescanBack[i][j] - prescanReference[i][j]);
      } else {
        pxSkipped++;
      }
    }
  }
  if ((sqrt((double)inc) / (double)(prescanReference.getSize() - pxSkipped)) <
      0.5) // we allow a mean difference of 1 units per pixels between the images
    testFailed = false;

  if (!testFailed)
    std::cout << "Test passed !" << std::endl;
  return testFailed;
}
#else
int main(int argc, const char **argv)
{
  (void)argc;
  (void)argv;
  std::cout << "You should intall xml2 to run this test\n";
  return 0;
}
#endif
