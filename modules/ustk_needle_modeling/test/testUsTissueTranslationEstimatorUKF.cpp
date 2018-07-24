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
  \example testUSTissueTranslationEstimatorUKF.cpp

  USTK usTissueTranslationEstimatorUKF test

  This example tests the update of the tissue representation of a class usNeedleInsertionModelRayleighRitzSpline using the estimator usTissueTranslationEstimatorUKF.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_LAPACK)

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
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/ustk_needle_modeling/usNeedleModelingDisplayTools.h>
#include <visp3/ustk_needle_modeling/usNeedleInsertionModelRayleighRitzSpline.h>
#include <visp3/ustk_needle_modeling/usTissueTranslationEstimatorUKF.h>

// List of allowed command line options
#define GETOPTARGS "hlt:cd"

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
Tests the class usTissueTranslationEstimatorUKF.\n\
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
        display1->init(I1, 0,0, "XZ");
        display2->init(I2, display1->getWindowXPosition()+display1->getWidth(), display1->getWindowYPosition(), "YZ");
    }
    
    std::cout << "Start test testUsTissueTranslationEstimatorUKF" << std::endl;

    const int nbNeedles = 6;
    usNeedleInsertionModelRayleighRitzSpline needleRef;
    usNeedleInsertionModelRayleighRitzSpline needle[nbNeedles];

    needleRef.loadPreset(usNeedleInsertionModelRayleighRitzSpline::ModelPreset::BiopsyNeedle);
    for(int i = 0 ; i<nbNeedles ; i++) needle[i].loadPreset(usNeedleInsertionModelRayleighRitzSpline::ModelPreset::BiopsyNeedle);

    needleRef.setPathUpdateType(usNeedleInsertionModelRayleighRitzSpline::PathUpdateType::WithTipPosition);
    for(int i = 0 ; i<nbNeedles ; i++) needle[i].setPathUpdateType(usNeedleInsertionModelRayleighRitzSpline::PathUpdateType::WithTipPosition);

    needleRef.setStiffnessPerUnitLength(10000);
    for(int i = 0 ; i<nbNeedles ; i++) needle[i].setStiffnessPerUnitLength(10000);

    vpPoseVector needleInitialBasePose(0,0,-0.08, 0, 0, 0);
    needleRef.setBasePose(needleInitialBasePose);
    for(int i = 0 ; i<nbNeedles ; i++) needle[i].setBasePose(needleInitialBasePose);

    needleRef.setSurfaceAtTip();
    for(int i = 0 ; i<nbNeedles ; i++) needle[i].setSurfaceAtTip();

    vpPlot *statePlot = NULL;
    if(opt_display)
    {
        statePlot = new vpPlot;
        statePlot->init(4);
        for(int i=0 ; i<4 ; i++) statePlot->initGraph(i,1+nbNeedles);
        statePlot->setTitle(0, "Tissue position X");
        statePlot->setTitle(1, "Tissue position Y");
        statePlot->setTitle(2, "Tissue velocity X");
        statePlot->setTitle(3, "Tissue velocity Y");
        statePlot->setLegend(0,0, "Ref");
        for(int i=0 ; i<nbNeedles ; i++) statePlot->setLegend(0,i+1,[i](){std::ostringstream t;t<<i;return t.str();}());
    }

    double std_measure_position = 1e-3;
    double std_measure_tip_direction = 1e-3;
    double std_measure_force = 1e-3;
    double std_measure_torque = 1e-3;
    double std_process_position_only = 1e-4;
    double std_process_position = 1e-7;
    double std_process_velocity = 1e-5;

    usTissueTranslationEstimatorUKF Kalman[nbNeedles];
    for(int i=0 ; i<nbNeedles ; i++)
    {
        Kalman[i].setPositionMeasureNoiseVariance(std_measure_position*std_measure_position);
        Kalman[i].setTipDirectionMeasureNoiseVariance(std_measure_tip_direction*std_measure_tip_direction);
        Kalman[i].setForceMeasureNoiseVariance(std_measure_force*std_measure_force);
        Kalman[i].setTorqueMeasureNoiseVariance(std_measure_torque*std_measure_torque);

        Kalman[i].setProcessNoiseType(usTissueTranslationEstimatorUKF::ADDITIVE_NOISE);
        Kalman[i].setMeasureNoiseType(usTissueTranslationEstimatorUKF::ADDITIVE_NOISE);
        
        Kalman[i].setSigmaPointGenerationType(usTissueTranslationEstimatorUKF::LIMITED_SPREAD);
        Kalman[i].setSigmaPointScalingFactor(1e-2);
        Kalman[i].setSigmaPointSpreadThreshold(0.0001);
        
        Kalman[i].setTissueTranslationType(usTissueTranslationEstimatorUKF::LATERAL_TRANSLATIONS_ONLY);
    }
    
    for(int i=0 ; i<nbNeedles/2 ; i++)
    {
      Kalman[i].setStateDynamicsType(usTissueTranslationEstimatorUKF::CONSTANT_VELOCITY);
      vpMatrix P(6,6,0);
      P[3][3] = 0.0001*0.0001;
      P[4][4] = 0.0001*0.0001;
      Kalman[i].setStateCovarianceMatrix(P);
      Kalman[i].setTissuePositionProcessNoiseVariance(std_process_position*std_process_position);
      Kalman[i].setTissueVelocityProcessNoiseVariance(std_process_velocity*std_process_velocity);
      
      Kalman[nbNeedles/2+i].setStateDynamicsType(usTissueTranslationEstimatorUKF::CONSTANT_POSITION);
      P.resize(3,3);
      Kalman[nbNeedles/2+i].setStateCovarianceMatrix(P);
      Kalman[nbNeedles/2+i].setTissuePositionProcessNoiseVariance(std_process_position_only*std_process_position_only);
    }
    
    for(int i=0 ; i<nbNeedles ; i+=3)
    {
      Kalman[i].setMeasureType(usTissueTranslationEstimatorUKF::NEEDLE_BODY_POINTS);
      Kalman[i+1].setMeasureType(usTissueTranslationEstimatorUKF::TIP_POSITION_AND_DIRECTION);
      Kalman[i+2].setMeasureType(usTissueTranslationEstimatorUKF::BASE_FORCE_TORQUE);
    }
      
    vpGaussRand GR_points(std_measure_position,0,(long)vpTime::measureTimeMs());
    vpGaussRand GR_tip_dir(std_measure_tip_direction,0,(long)vpTime::measureTimeMs());
    vpGaussRand GR_force(std_measure_force,0,(long)vpTime::measureTimeMs());
    vpGaussRand GR_torque(std_measure_torque,0,(long)vpTime::measureTimeMs());

    double time = 0;
    for(int i=0; i<500 ;i++)
    {
        for(int i=0 ; i<nbNeedles ; i++) Kalman[i].setCurrentNeedle(needle[i]);

        if(i<100)
        {
            vpColVector baseControl(6,0);
            baseControl[2] = 0.0005;
            baseControl[1] = 0.00005;
            needleRef.moveBase(baseControl, 1);
            for(int i=0 ; i<nbNeedles ; i++) needle[i].moveBase(baseControl, 1);
        }

        if(i<100)
        {
            needleRef.accessTissue().move(0.0001,0,0,0,0,0);
            needleRef.updateState();
            if(opt_display)
            {
                statePlot->plot(2,0, time, 0.0001);
                statePlot->plot(3,0, time, 0);
            }
        }
        else if(i<200)
        {
            needleRef.accessTissue().move(0,0.0001,0,0,0,0);
            needleRef.updateState();
            if(opt_display)
            {
                statePlot->plot(2,0, time, 0);
                statePlot->plot(3,0, time, 0.0001);
            }
        }
        else if(i<300)
        {
            needleRef.accessTissue().move(-0.0002,0,0,0,0,0);
            needleRef.updateState();
            if(opt_display)
            {
                statePlot->plot(2,0, time, -0.0002);
                statePlot->plot(3,0, time, 0);
            }
        }
        else if(i<400)
        {
            needleRef.accessTissue().move(0,-0.0002,0,0,0,0);
            needleRef.updateState();
            if(opt_display)
            {
                statePlot->plot(2,0, time, 0);
                statePlot->plot(3,0, time, -0.0002);
            }
        }
        else
        {
            needleRef.accessTissue().move(0.0001,0.0001,0,0,0,0);
            needleRef.updateState();
            if(opt_display)
            {
                statePlot->plot(2,0, time, 0.0001);
                statePlot->plot(3,0, time, 0.0001);
            }
        }
        
        if(opt_display)
        {
            statePlot->plot(0,0, time, needleRef.accessTissue().getPose()[0]);
            statePlot->plot(1,0, time, needleRef.accessTissue().getPose()[1]);
        }

        int nbObs = floor(needleRef.getInsertionDepth()/0.005) + 2;
        double step = needleRef.getInsertionDepth()/(nbObs-1);
        vpColVector CP(3*nbObs);

        for(int j=0 ; j<nbObs ; j++)
        {
            vpColVector p = needleRef.accessNeedle().getNeedlePoint(needleRef.accessNeedle().getParametricLength()-j*step);
            for(int dim=0 ; dim<3 ; dim++) p[dim] += GR_points();
            CP.insert(3*j, p);
        }
        
        vpColVector tipMeasures(6);
        tipMeasures.insert(0, needleRef.accessNeedle().getTipPosition());
        tipMeasures.insert(3, needleRef.accessNeedle().getTipDirection());
        for(int dim=0 ; dim<3 ; dim++)
        { 
            tipMeasures[dim] += GR_points();
            tipMeasures[3+dim] += GR_tip_dir();
        }
        
        vpColVector baseForceMeasures(needleRef.accessNeedle().getBaseStaticTorsor());
        for(int dim=0 ; dim<3 ; dim++)
        { 
            baseForceMeasures[dim] += GR_force();
            baseForceMeasures[3+dim] += GR_torque();
        }

        for(int i=0 ; i<nbNeedles ; i++) Kalman[i].setPropagationTime(1.);
        
        for(int i=0 ; i<nbNeedles ; i+=3)
        {
            Kalman[i].filter(CP);
            Kalman[i+1].filter(tipMeasures);
            Kalman[i+2].filter(baseForceMeasures);
        }

        for(int i=0 ; i<nbNeedles ; i++)
        {
            Kalman[i].applyStateToNeedle(needle[i]);
            needle[i].updateState();
            if((vpColVector(needleRef.accessTissue().getPose(), 0,3) - vpColVector(needle[i].accessTissue().getPose(), 0,3)).euclideanNorm() > 0.002)
            {
                std::cout << "bad estimation for needle " << i << std::endl;
                return 1;
            }
        }

        if(opt_display)
        {
            vpDisplay::display(I1);
            vpDisplay::display(I2);
            usNeedleModelingDisplayTools::display(needleRef, I1, vpHomogeneousMatrix(0.08, 0.1, 0.2, -M_PI / 2, 0, 0), 3000, 3000);
            usNeedleModelingDisplayTools::display(needleRef, I2, vpHomogeneousMatrix(0.08, 0.1, 0.2, -2*M_PI / (3*sqrt(3)), 2*M_PI / (3*sqrt(3)), 2*M_PI / (3*sqrt(3))), 3000, 3000);
            for(int i=0 ; i<nbNeedles ; i++)
            {
                usNeedleModelingDisplayTools::display(needle[i], I1, vpHomogeneousMatrix(0.08, 0.1, 0.2, -M_PI / 2, 0, 0), 3000, 3000);
                usNeedleModelingDisplayTools::display(needle[i], I2, vpHomogeneousMatrix(0.08, 0.1, 0.2, -2*M_PI / (3*sqrt(3)), 2*M_PI / (3*sqrt(3)), 2*M_PI / (3*sqrt(3))), 3000, 3000);

                statePlot->plot(0,1+i, time, needle[i].accessTissue().getPose()[0]);
                statePlot->plot(1,1+i, time, needle[i].accessTissue().getPose()[1]);
            }
            for(int i=0 ; i<nbNeedles/2 ; i++)
            {
                statePlot->plot(2,1+i, time, Kalman[i].getState()[3]);
                statePlot->plot(3,1+i, time, Kalman[i].getState()[4]);
            }
            vpDisplay::flush(I1);
            vpDisplay::flush(I2);
        }
        time += 1;
    }
    
    if(opt_display)
    {
      delete display1;
      delete display2;
      delete statePlot;
    }

    return 0;
}

#else

#include <iostream>

int main()
{
  std::cout << "testUSTissueTranslationEstimatorUKF cannot be run without Eigen3 or Lapack" << std::endl;
  return 0;
}

#endif
