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
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  
#	Example file for needle detection.
#
#       Pierre Chatelain
#       July 10, 2015
#
#----------------------------------------------------------------------------
*/


// visp
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/gui/vpPlot.h>

//ustk
#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/ustk_needle_detection/usNeedleTrackerSIR2D.h>

using namespace std;

/* -------------------------------------------------------------------------- */
/*                         COMMAND LINE OPTIONS                               */
/* -------------------------------------------------------------------------- */

// List of allowed command line options
#define GETOPTARGS	"cdo:h"

void usage(const char *name, const char *badparam, const std::string& opath, const std::string& user);
bool getOptions(int argc, const char **argv, std::string &opath, const std::string& user);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param opath : Output image path.
\param user : Username.

 */
void usage(const char *name, const char *badparam, const std::string& opath, const std::string& user)
{
  fprintf(stdout, "\n\
          Write and read ultrasound sequences in 2d image files, and the associated xml settings file.\n\
          \n\
          SYNOPSIS\n\
          %s [-o <output image path>] [-h]\n", name);

      fprintf(stdout, "\n\
              OPTIONS:                                               Default\n\
              -o <output data path>                               %s\n\
              Set data output path.\n\
              From this directory, creates the \"%s\"\n\
              subdirectory depending on the username, where \n\
              sequenceRF2D.xml file is written.\n\
              \n\
              -h\n\
              Print the help.\n\n", opath.c_str(), user.c_str());

              if (badparam) {
                fprintf(stderr, "ERROR: \n" );
                fprintf(stderr, "\nBad parameter [%s]\n", badparam);
              }
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param opath : Output data path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &opath, const std::string& user)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'o': opath = optarg_; break;
    case 'h': usage(argv[0], NULL, opath, user); return false; break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}


int main(int argc, const char *argv[])
{
  std::string opt_opath;
  std::string username;

  char *logFilename = new char [FILENAME_MAX];
  const char *opath = new char [FILENAME_MAX];
  char *windowTitle = new char[32];

  // Set the default output path
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  opt_opath = "/tmp";
#elif defined(_WIN32)
  opt_opath = "C:\\temp";
#endif

  // Get the user login name
  vpIoTools::getUserName(username);

  // Read the command line options
  if (getOptions(argc, argv, opt_opath, username) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_opath.empty())
    opath = opt_opath.c_str();

  // Append to the output path string, the login name of the user
  std::string dirname = vpIoTools::createFilePath(opath, username);

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(dirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(dirname);
    }
    catch (...) {
      usage(argv[0], NULL, opath, username);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << dirname << std::endl;
      std::cerr << "  Check your -o " << opath << " option " << std::endl;
      delete[] opath;
      exit(-1);
    }
  }


  std::string xml_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      xml_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <needleSequence.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (xml_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      xml_filename = env_ipath + "/needle/water_bath_minimal_noise_png/sequence.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }



  ///////////////////////////////////////////////////////////////////////
  //
  // Initializations.
  //
  ///////////////////////////////////////////////////////////////////////

  usSequenceReader<usImagePostScan2D<unsigned char> > reader;
  reader.setSequenceFileName(xml_filename);

  // Read the first image
  usImagePostScan2D<unsigned char> I;
  reader.acquire(I);

  int n0 = 150;

#if defined VISP_HAVE_X11
  vpDisplay *display = new vpDisplayX(I);
#elif defined VISP_HAVE_GTK
  vpDisplay *display = new vpDisplayGTK(I);
#elif defined VISP_HAVE_GDI
  vpDisplay *display = new vpDisplayGDI(I);
#elif defined VISP_HAVE_D3D9
  vpDisplay *display = new vpDisplayD3D(I);
#elif defined VISP_HAVE_OPENCV
  vpDisplay *display = new vpDisplayOpenCV(I);
#else
#error "No display available."
#endif 

  vpDisplay::display(I);
  vpDisplay::flush(I);

  // Initialize the needle model
  usPolynomialCurve2D needle(2);
  vpMatrix controlPoints(2, 2);
  vpImagePoint P;

  std::cout << "Click on the entry point." << std::endl;
  vpDisplay::getClick(I, P);

  controlPoints[0][0] = P.get_i();
  controlPoints[1][0] = P.get_j();

  std::cout << "Click on the tip." << std::endl;
  vpDisplay::getClick(I, P);
  
  controlPoints[0][1] = P.get_i();
  controlPoints[1][1] = P.get_j();
  
  needle.setControlPoints(controlPoints.t());
  std::cout << "Needle model initialized." << std::endl;

  // Initialization of the needle detector
  usNeedleTrackerSIR2D needleDetector;

  unsigned int nControlPoints = 2;
  unsigned int nParticles = 200;

  needleDetector.setSigma(1.0);
  needleDetector.setSigma1(10.0);
  needleDetector.setSigma2(1.0);
  needleDetector.init(I, nControlPoints, nParticles, needle);
  std::cout << "Needle detector initialized." << std::endl;

  // Output
  sprintf(logFilename, "%s/needle.dat", dirname.c_str());
  std::ofstream ofile(logFilename);
  std::cout << "Results will be saved in " << logFilename << std::endl;

  unsigned int nPoints =  needleDetector.getNeedle()->getOrder();
  controlPoints = needleDetector.getNeedle()->getControlPoints();
  ofile << nPoints;
  for (unsigned int i = 0; i < nPoints; ++i)
    ofile << " " << controlPoints[0][i]
             << " " << controlPoints[1][i];
  ofile << std::endl;
  
  vpColVector tipPose, entryPose;

  ///////////////////////////////////////////////////////////////////////
  //
  // Start needle detection.
  //
  ///////////////////////////////////////////////////////////////////////

  //printf(logFilename, "%05d.%s", n0++, extension.c_str());

  unsigned int it = 1;
  vpMatrix tipStd(2, 2);
  vpColVector tipMean;
  vpColVector evalue, evector;
  vpMatrix rendering;

  while (!reader.end()) {
    reader.acquire(I);
    needleDetector.run(I, 0.0);

    tipMean = needleDetector.getNeedle()->getPoint(1.0);
    entryPose = needleDetector.getNeedle()->getPoint(0.0);
    cout << "Tip position: (" << tipMean[0] << "," << tipMean[1]
         << ")" << endl;
    cout << "Needle length: " << needleDetector.getNeedle()->getLength() << endl;
    cout << "Number of control points: " << needleDetector.getNeedle()->getOrder()
         << endl;

    // Output
    nPoints =  needleDetector.getNeedle()->getOrder();
    controlPoints = needleDetector.getNeedle()->getControlPoints();
    ofile << nPoints;
    for (unsigned int i = 0; i < nPoints; ++i)
      ofile << " " << controlPoints[0][i]
               << " " << controlPoints[1][i];
    ofile << std::endl;

    // Display
    std::sprintf(windowTitle, "Frame %d", n0);
    vpDisplay::setTitle(I, windowTitle);
    vpDisplay::display(I);

    rendering = needleDetector.getNeedle()->getRenderingPoints();
    unsigned int n = rendering.getCols();
    
    for (unsigned int j = 0; j < n - 1; ++j)
      vpDisplay::displayLine(I, rendering[0][j], rendering[1][j],
          rendering[0][j+1], rendering[1][j+1],
          vpColor::red, 2);

    tipStd = 0.0;

    for (unsigned int i = 0; i < nParticles; ++i) {
      tipPose = needleDetector.getParticle(i)->getPoint(1.0);
      tipStd += needleDetector.getWeight(i)
          * (tipPose - tipMean) * (tipPose - tipMean).t();

      if ((it % 10) == 0)
        vpDisplay::displayCross(I, tipPose[0], tipPose[1], 3, vpColor::blue);
    }

    vpDisplay::flush(I);

    //sprintf(logFilename, "%05d.%s", n0++, extension.c_str());

    //vpIoTools::setBaseName(logFilenames);
    ++it;
  }

  //delete and close everything
#if defined VISP_HAVE_DISPLAY
  delete display;
#endif
  ofile.close();
  exit(0);
  return 0;
}
