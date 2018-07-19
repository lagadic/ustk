/****************************************************************************
 *
 * This file is part of the UsNeedleDetection software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

/*!
  \example testUsGeometryDisplayTools.cpp

  USTK usGeometryDisplayTools test

  This example tests all functions declared in the usGeometryDisplayTools namespace.
*/

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/io/vpParseArgv.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_core/usGeometryDisplayTools.h>

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
Test the display functions in usGeometryDisplayTools.\n\
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

    vpImage<unsigned char> I1(700, 500, 255);
    vpImage<vpRGBa> I2(700, 500, vpRGBa(255,255,255,255));
    
    vpDisplay *display1 = nullptr;
    vpDisplay *display2 = nullptr;

    switch (opt_dtype)
    {
        case vpX11:
            std::cout << "Requested X11 display functionnalities..." << std::endl;
#if defined VISP_HAVE_X11
            display1 = new vpDisplayX;
            display2 = new vpDisplayX;
#else
            std::cout << "  Sorry, X11 video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpGTK:
            std::cout << "Requested GTK display functionnalities..." << std::endl;
#if defined VISP_HAVE_GTK
            display1 = new vpDisplayGTK;
            display2 = new vpDisplayGTK;
#else
            std::cout << "  Sorry, GTK video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpGDI:
            std::cout << "Requested GDI display functionnalities..." << std::endl;
#if defined VISP_HAVE_GDI
            display1 = new vpDisplayGDI;
            display2 = new vpDisplayGDI;
#else
            std::cout << "  Sorry, GDI video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpD3D:
            std::cout << "Requested D3D display functionnalities..." << std::endl;
#if defined VISP_HAVE_D3D9
            display1 = new vpDisplayD3D;
            display2 = new vpDisplayD3D;
#else
            std::cout << "  Sorry, D3D video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
        case vpCV:
            std::cout << "Requested OpenCV display functionnalities..." << std::endl;
#if defined(VISP_HAVE_OPENCV)
            display1 = new vpDisplayOpenCV;
            display2 = new vpDisplayOpenCV;
#else
            std::cout << "  Sorry, OpenCV video device is not available.\n";
            std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
            return 0;
#endif
            break;
    }

    if(opt_display)
    {
        display1->init(I1, 0,0, "Display unsigned char");
        display2->init(I2, display1->getWindowXPosition()+display1->getWidth(), display1->getWindowYPosition(), "Display vpRGBa");
    }
    
    std::cout << "Start test testUsGeometryDisplayTools" << std::endl;
        
    usOrientedPlane3D plane(vpPoseVector(0,0,0.05, 0,M_PI/4,0));
    
    std::vector<vpColVector> points2D(4, vpColVector(2,0));
    points2D.at(0)[0] = 0.02;points2D.at(0)[1] = 0.02;
    points2D.at(1)[0] = 0.02;points2D.at(1)[1] = 0.03;
    points2D.at(2)[0] = 0.03;points2D.at(2)[1] = 0.03;
    points2D.at(3)[0] = 0.02;points2D.at(3)[1] = 0.05;
    usPolynomialCurve2D curve2D;
    curve2D.defineFromPointsAuto(points2D, points2D.back()-points2D.front(),3);

    std::vector<vpColVector> points3D(4, vpColVector(3,0));
    points3D.at(1)[0] = 0.02;
    points3D.at(2)[1] = 0.02;points3D.at(2)[2] = 0.03;
    points3D.at(3)[2] = 0.05;
    usPolynomialCurve3D curve3D;
    curve3D.defineFromPointsAuto(points3D, points3D.back()-points3D.front(),3);
        
    usBSpline3D spline;
    std::vector<double> lengths(3);
    lengths.at(0) = 0.02;lengths.at(1) = 0.04;lengths.at(2) = 0.04;
    spline.defineFromPoints(points3D, lengths, 3);
    
    vpHomogeneousMatrix imageMworld(0.1, 0.1, 0.1, M_PI/2, 0, 0);

    vpDisplay::display(I1);
    vpDisplay::display(I2);
    
    usGeometryDisplayTools::display(plane, I1, imageMworld, 3000, 3000, vpColor::green);
    std::cout << "done: usGeometryDisplayTools::display(const usOrientedPlane3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::display(plane, I2, imageMworld, 3000, 3000, vpColor::green);
    std::cout << "done: usGeometryDisplayTools::display(const usOrientedPlane3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    
    usGeometryDisplayTools::display(curve2D, I1, 3000, 3000, vpColor::red, 10);
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve2D&, vpImage<unsigned char>&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::display(curve2D, I2, 3000, 3000, vpColor::red, 10);
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve2D&, vpImage<vpRGBa>&, double, double, const vpColor&);" << std::endl;
    
    usGeometryDisplayTools::display(curve3D, I1, imageMworld, 3000, 3000, vpColor::blue, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    usGeometryDisplayTools::display(curve3D, I2, imageMworld, 3000, 3000, vpColor::blue, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::display(const usPolynomialCurve3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    
    usGeometryDisplayTools::displayLine(spline, I1, imageMworld, 3000, 3000, vpColor::orange, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayLine(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    usGeometryDisplayTools::displayLine(spline, I2, imageMworld, 3000, 3000, vpColor::orange, 10, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayLine(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, int, double);" << std::endl;
    
    usGeometryDisplayTools::displayExtremities(spline, I1, imageMworld, 3000, 3000, vpColor::orange, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayExtremities(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);" << std::endl;
    usGeometryDisplayTools::displayExtremities(spline, I2, imageMworld, 3000, 3000, vpColor::orange, std::numeric_limits<double>::infinity());
    std::cout << "done: usGeometryDisplayTools::displayExtremities(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&, double);" << std::endl;
    
    usGeometryDisplayTools::displayCurvatureFromShape(spline, I1, imageMworld, 3000, 3000, vpColor::orange);
    std::cout << "done: usGeometryDisplayTools::displayCurvatureFromShape(const usBSpline3D&, vpImage<unsigned char>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    usGeometryDisplayTools::displayCurvatureFromShape(spline, I2, imageMworld, 3000, 3000, vpColor::orange);
    std::cout << "done: usGeometryDisplayTools::displayCurvatureFromShape(const usBSpline3D&, vpImage<vpRGBa>&, const vpHomogeneousMatrix&, double, double, const vpColor&);" << std::endl;
    
    vpDisplay::flush(I1);
    vpDisplay::flush(I2);
    
    vpTime::wait(2000);
    
    delete display1;
    delete display2;
    
    return 0;
}
