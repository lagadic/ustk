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
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/io/vpImageIo.h>
#include <visp3/ustk_elastography/usElastography.h>
#include <visp3/ustk_io/usImageIo.h>

int main()
{

  // prepare image;
  usImageRF2D<short int> preComp;
  usImageRF2D<short int> postComp;

  std::string image1 = us::getDataSetPath() + std::string("/RFElasto/image00012.mhd");
  std::string image2 = us::getDataSetPath() + std::string("/RFElasto/image00015.mhd");

  usImageIo::read(preComp, image1.c_str());
  usImageIo::read(postComp, image2.c_str());

  usElastography elastography(preComp, postComp);
  elastography.setROI(40, 2500, 50, 500);

  // computate elasto
  vpImage<unsigned char> strainImage;
  strainImage = elastography.run();

  // read groud truth image
  vpImage<unsigned char> groundTruthStrainImage;
  std::string groundTruthFileName =
      us::getDataSetPath() + std::string("/ustk-tests-groudTruth/testElasto_imgRFElasto12&15_ROI-40-2500-50-500.png");
  vpImageIo::read(groundTruthStrainImage, groundTruthFileName.c_str());

  // compare
  if (strainImage == groundTruthStrainImage)
    return 0;

  std::cout << "Test failed !\n";
  return 1;
}

#else
int main()
{
  std::cout << "You should intall FFTW to run this test" << std::endl;
  return 0;
}

#endif
