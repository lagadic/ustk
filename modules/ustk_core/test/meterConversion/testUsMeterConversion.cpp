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

#include <visp3/ustk_core/usMeterPixelConversion.h>
#include <visp3/ustk_core/usPixelMeterConversion.h>

#include <string>
#include <vector>

/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */

int main(int argc, const char** argv)
{
    bool testFailed = false;
    if (argc>4) {
      std::cout << "Wrong number of arguments";
      exit (-1);
    }
    std::cout << "Usage : ./testUsMeterConversion 4 5 6 "<<std::endl;
    std::cout << "In this case you're going to convert pixel at coordinates (4,5,6) in (u,v,w) base." <<std::endl;
    std::cout << "If no argument provided default values are set" <<std::endl;

    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout <<  "  testUsMeterConversion.cpp" <<std::endl << std::endl ;
    std::cout <<  "  The test converts a pixel position to a meter position, and converts it back to pixel conversion."
                  "  Then it checks if it is the same position." << std::endl ;
    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;

    double UBase = 0.0;
    double VBase = 0.0;
    //For 3D
    //double WBase = 0.0;
    double X, Y, U, V;

    if(argc>=2)
      UBase = atof(argv[1]);
    if(argc>=3)
      VBase = atof(argv[2]);
    //For 3D
    //if(argc>=4)
      //WBase = atof(argv[3]);

    //First test for 2d image with convex transducer
    std::cout << "Test for 2D image with convex transducer" << std::endl;

    vpImage<unsigned char> data(186, 233, 128); // Set pixel intensity to 128
    usImagePostScan2D<unsigned char> usImagePostScan2DReference;
    usImagePostScan2DReference.setData(data);
    usImagePostScan2DReference.setScanLinePitch(0.0145);
    usImagePostScan2DReference.setScanLineNumber(128);
    usImagePostScan2DReference.setTransducerRadius(0.0554);
    usImagePostScan2DReference.setTransducerConvexity(true);
    usImagePostScan2DReference.setWidthResolution(0.00158);
    usImagePostScan2DReference.setHeightResolution(0.0018);

    std::cout << "before conversion, u = " << std::scientific << UBase << ", and v = " << std::scientific << VBase << std::endl;

    usPixelMeterConversion::convert(usImagePostScan2DReference,UBase,VBase,X,Y);

    std::cout << "after conversion, x = " << std::scientific << X << ", and y = " << std::scientific << Y << std::endl;

    usMeterPixelConversion::convert(usImagePostScan2DReference,X,Y,U,V);

    std::cout << "after back conversion, u = " << std::scientific << U << ", and v = " << std::scientific << V << std::endl;

    std::cout << "|U - UBase| = " << std::abs(U - UBase) << std::endl;
    std::cout << "|UBase * epsilon| = " << std::abs(std::numeric_limits<double>::epsilon()*UBase)  << std::endl;
    std::cout << "|V - VBase| = " << std::abs(V - VBase) << std::endl;
    std::cout << "|VBase * epsilon| = " << std::abs(std::numeric_limits<double>::epsilon()*VBase)  << std::endl;

    if(!(std::abs(U - UBase) <= std::abs(std::numeric_limits<double>::epsilon()*UBase) &&
         std::abs(V - VBase) <= std::abs(std::numeric_limits<double>::epsilon()*VBase) )) {
      std::cout << "ERROR : Test for 2d image with convex transducer failed !" << std::endl;
      testFailed = true;
    }


    //Second test for 2d image with linear transducer
    std::cout << "Test for 2D image with linear transducer" << std::endl;

    usImagePostScan2DReference.setTransducerConvexity(false);

    std::cout << "before conversion, u = " << std::scientific << UBase << ", and v = " << std::scientific << VBase << std::endl;

    usPixelMeterConversion::convert(usImagePostScan2DReference,UBase,VBase,X,Y);

    std::cout << "after conversion, x = " << std::scientific << X << ", and y = " << std::scientific << Y << std::endl;

    usMeterPixelConversion::convert(usImagePostScan2DReference,X,Y,U,V);

    std::cout << "after back conversion, u = " << std::scientific << U << ", and v = " << std::scientific << V << std::endl;

    std::cout << "|U - UBase| = " << std::abs(U - UBase) << std::endl;
    std::cout << "|UBase * epsilon| = " << std::abs(std::numeric_limits<double>::epsilon()*UBase)  << std::endl;
    std::cout << "|V - VBase| = " << std::abs(V - VBase) << std::endl;
    std::cout << "|VBase * epsilon| = " << std::abs(std::numeric_limits<double>::epsilon()*VBase)  << std::endl;

    if(!(std::abs(U - UBase) <= std::abs(std::numeric_limits<double>::epsilon()*UBase) &&
         std::abs(V - VBase) <= std::abs(std::numeric_limits<double>::epsilon()*VBase) )) {
      std::cout << "ERROR : Test for 2d image with linear transducer failed !" << std::endl;
      testFailed = true;
    }

    if(!testFailed)
      std::cout << "Test passed !" << std::endl;
    return testFailed;
}

