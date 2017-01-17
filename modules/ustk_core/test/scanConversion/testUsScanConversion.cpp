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

#include <visp3/ustk_core/usScanConverter2D.h>
#include <visp3/ustk_core/usBackScanConverter2D.h>

#include <string>
#include <vector>

/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */

int main(int argc, const char** argv)
{
    (void) argc;
    (void) argv;

    /*bool testFailed = false;
    if (argc!=2) {
      std::cout << "Wrong number of arguments" << argc << std::endl;
      exit (0); //to change when we have a dataset to provide for this test
    }
    std::cout << "Usage : ./testUsScanConversion2D /path/to/prescan2d "<<std::endl;
    std::cout << "In this case you're going to convert the pre-scan image into post-scan,"
                 " and then convert it back to pre-scan." <<std::endl;

    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout <<  "  testUsScanConversion2D.cpp" <<std::endl << std::endl ;
    std::cout <<  "  The test converts a pre-scan image to post-scan, and converts it back to pre-scan."
                  "  Then it checks if the 2 pre-scan images are equal." << std::endl ;
    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;

    //pre-scan reading
    usImagePreScan2D<unsigned char> prescanReference;
    usImagePostScan2D<unsigned char> postscan;
    usImagePreScan2D<unsigned char> prescanBack;

    usImageIo::read(prescanReference, argv[1]);
    prescanReference.setDepth(0.14784);

    usScanConverter2D scanConverter;
    scanConverter.init(prescanReference.getBModeSampleNumber(),prescanReference.getScanLineNumber(),
                       ustk::defaultSpeedOfSound, 0.0005,prescanReference.getTransducerRadius(),2500000.0,0.00042168,128);
    scanConverter.run(postscanReference,prescanReference);*/
                       /*
radius 0.04
depth 0.14784
APitch 0.000308
LPitch 0.000425
resolution 0.0005
AN 480
LN 128

  m_APitch = m_speedOfSound / (2.0 * samplingFrequency); => frequ = speed/(2*apitch) = 1540/0.000616 = 2500000
  m_LPitch = (pitch * nElements) / (LN-1); => pitch*NElts = Lpitch * (LN-1) = 0.000425 * 127 = 0.053975
                                          => nELts = 128 && pitch = 0.00042168


  */
    /*postscanReference.setDepth(0.14784);
    usImageIo::write(postscanReference,vpIoTools::getParent(argv[1]).append("/aaa.xml"));


    usBackScanConverter2D backScanConverter;
    backScanConverter.init(postscanReference,480,128);
    backScanConverter.run(postscanReference,prescanBack);

    usImageIo::write(prescanBack,vpIoTools::getParent(argv[1]).append("/bbb.xml"));

    if(!testFailed)
      std::cout << "Test passed !" << std::endl;*/
    return 0;
}

