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
 * Jason Chevrie
 *
 *****************************************************************************/

/*!
  \example testUsNeedleInsertionModelRayleighRitzSpline.cpp

  USTK usNeedleInsertionModelRayleighRitzSpline test

  This example tests the model of the insertion of a needle via the class usNeedleInsertionModelRayleighRitzSpline.
*/

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <stdlib.h>
#include <string>

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/io/vpParseArgv.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoseVector.h>

#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usNeedleModelingDisplayTools.h>

// List of allowed command line options
#define GETOPTARGS "hlt:d"

typedef enum { vpX11, vpGTK, vpGDI, vpD3D, vpCV } vpDisplayType;

void usage(const char *name, const char *badparam, vpDisplayType &dtype);
bool getOptions(int argc, const char **argv, vpDisplayType &dtype, bool &list, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param dtype : Type of video device.

 */
void usage(const char *name, const char *badparam, vpDisplayType &dtype)
{
  fprintf(stdout, "\n\
Test the class usNeedleInsertionModelRayleighRitzSpline.\n\
\n\
SYNOPSIS\n\
  %s [-t <type of video device>] [-l] [-d] [-h]\n\
", name);

  std::string display;
  switch (dtype) {
  case vpX11:
    display = "X11";
    break;
  case vpGTK:
    display = "GTK";
    break;
  case vpGDI:
    display = "GDI";
    break;
  case vpD3D:
    display = "D3D";
    break;
  case vpCV:
    display = "CV";
    break;
  }

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -t <type of video device>                            \"%s\"\n\
     String specifying the video device to use.\n\
     Possible values:\n\
       \"X11\": only on UNIX platforms,\n\
       \"GTK\": on all plaforms,\n\
       \"GDI\": only on Windows platform (Graphics Device Interface),\n\
       \"D3D\": only on Windows platform (Direct3D).\n\
       \"CV\" : (OpenCV).\n\
\n\
  -l\n\
     Print the list of video-devices available and exit.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n\n", display.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param dtype : Type of video device.
  \param list : Set as true,list the available video-devices.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, vpDisplayType &dtype, bool &list, bool &display)
{
    const char *optarg_;
    int c;
    std::string sDisplayType;
    while((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1)
    {

        switch (c)
        {
            case 'l':
                list = true;
                break;
                
            case 't':
                sDisplayType = optarg_;
                // Parse the display type option
                if(sDisplayType.compare("X11") == 0) dtype = vpX11;
                else if (sDisplayType.compare("GTK") == 0) dtype = vpGTK;
                else if (sDisplayType.compare("GDI") == 0) dtype = vpGDI;
                else if (sDisplayType.compare("D3D") == 0) dtype = vpD3D;
                else if (sDisplayType.compare("CV") == 0) dtype = vpCV;
                break;
                
            case 'h':
                usage(argv[0], NULL, dtype);
                return false;
                break;

            case 'c':
                break;

            case 'd':
                display = false;
                break;

            default:
                usage(argv[0], optarg_, dtype);
                return false;
                break;
        }
    }

    if((c == 1) || (c == -1))
    {
        // standalone param or error
        usage(argv[0], NULL, dtype);
        std::cerr << "ERROR: " << std::endl;
        std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
        return false;
    }

    return true;
}



int main(int argc, const char **argv)
{
    bool opt_list = false;   // To print the list of video devices
    vpDisplayType opt_dtype; // Type of display to use
    bool opt_display = true;

// Default display is one available
#if defined VISP_HAVE_GTK
    opt_dtype = vpGTK;
#elif defined VISP_HAVE_X11
    opt_dtype = vpX11;
#elif defined VISP_HAVE_GDI
    opt_dtype = vpGDI;
#elif defined VISP_HAVE_D3D9
    opt_dtype = vpD3D;
#elif defined VISP_HAVE_OPENCV
    opt_dtype = vpCV;
#endif

    // Read the command line options
    if(!getOptions(argc, argv, opt_dtype, opt_list, opt_display)) exit(-1);

    // Print the list of video-devices available
    if (opt_list) {
      unsigned nbDevices = 0;
      std::cout << "List of video-devices available: \n";
#if defined VISP_HAVE_GTK
      std::cout << "  GTK (use \"-t GTK\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_X11
      std::cout << "  X11 (use \"-t X11\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_GDI
      std::cout << "  GDI (use \"-t GDI\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_D3D9
      std::cout << "  D3D (use \"-t D3D\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_OPENCV
      std::cout << "  CV (use \"-t CV\" option to use it)\n";
      nbDevices++;
#endif
      if (!nbDevices) {
        std::cout << "  No display is available\n";
      }
      return (0);
    }
        
    vpImage<unsigned char> I(700, 500, 255);
    
    vpDisplay *display = nullptr;

    switch (opt_dtype)
    {
        case vpX11:
            std::cout << "Requested X11 display functionnalities..." << std::endl;
#if defined VISP_HAVE_X11
            display = new vpDisplayX;
#else
            std::cout << "  Sorry, X11 video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpGTK:
            std::cout << "Requested GTK display functionnalities..." << std::endl;
#if defined VISP_HAVE_GTK
            display = new vpDisplayGTK;
#else
            std::cout << "  Sorry, GTK video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpGDI:
            std::cout << "Requested GDI display functionnalities..." << std::endl;
#if defined VISP_HAVE_GDI
            display = new vpDisplayGDI;
#else
            std::cout << "  Sorry, GDI video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpD3D:
            std::cout << "Requested D3D display functionnalities..." << std::endl;
#if defined VISP_HAVE_D3D9
            display = new vpDisplayD3D;
#else
            std::cout << "  Sorry, D3D video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpCV:
            std::cout << "Requested OpenCV display functionnalities..." << std::endl;
#if defined(VISP_HAVE_OPENCV)
            display = new vpDisplayOpenCV;
#else
            std::cout << "  Sorry, OpenCV video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
    }

    if(opt_display) display->init(I);
    
    usNeedleInsertionModelRayleighRitzSpline n;

    n.loadPreset(usNeedleInsertionModelRayleighRitzSpline::ModelPreset::BiopsyNeedle);
    n.accessNeedle().setOuterDiameter(0.001);
    n.accessNeedle().setInsideDiameter(0.0007);
    n.accessNeedle().setNeedleYoungModulus(75e9);
    n.accessNeedle().setFullLength(0.1);
    
    n.accessNeedle().accessSegment(0).setOrder(3);
    n.setStiffnessPerUnitLength(0, 10000);
    
    n.setPathUpdateType(usNeedleInsertionModelRayleighRitzSpline::PathUpdateType::WithTipPosition);
    n.setPathUpdateLengthThreshold(0.001);
        
    n.setBasePose(vpPoseVector(0,0,0.1, M_PI,0,0));

    n.accessTissue().accessSurface().setPosition(n.accessNeedle().getTipPosition()+0.01*n.accessNeedle().getTipDirection());
    n.accessTissue().accessSurface().setDirection(n.accessNeedle().getTipDirection());

    n.setNeedleTipType(usNeedleInsertionModelRayleighRitzSpline::NeedleTipType::ActuatedTip);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setDiameter(2*n.accessNeedle().getOuterDiameter());
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setLength(0.005);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setTipAngleDeg(5);
    dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setSteeringAngleDeg(90);
    
    n.setSolvingMethod(usNeedleInsertionModelRayleighRitzSpline::SolvingMethod::FixedBeamLength);
    
    n.updateState();

    usNeedleInsertionModelRayleighRitzSpline n1(n);
    n1.moveBaseWorldFrame(0.01,0,0,0,0,0);
    
    for(int i=0 ; i<3000 ; i++)
    {        
        if(i > 500 && i < 1000)
        {
            n1.accessNeedle().setFullLength(n1.accessNeedle().getFullLength()+0.00002);
            n1.updateState();
        }
        
        if(i > 500)
        {
            dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setTipAngleDeg(0.1*(i%400));            
            dynamic_cast<usNeedleTipActuated&>(n.accessNeedleTip()).setSteeringAngleDeg(i%360);
            n.updateState();
        }

        n.moveBase(0,0,0.0001,0,0,0.01);
        n1.moveBase(0,0,0.0001,0,0,0.01);
        
        if(opt_display)
        {
            vpDisplay::display(I);
        
            usNeedleModelingDisplayTools::display(n, I, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
            usNeedleModelingDisplayTools::display(n1, I, vpHomogeneousMatrix(0.08 ,0.1, 0.2, M_PI/2,0,0), 3000,3000);
        
            vpDisplay::flush(I);
        }
    }

    if(display) delete display;

    return 0;
}
