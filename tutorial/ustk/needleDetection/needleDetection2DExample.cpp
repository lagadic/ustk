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

/*!
  Input: Camera images.
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

#include <visp3/ustk_needle_detection/usNeedleTrackerSIR2D.h>

using namespace std;


int main(int argc, const char *argv[])
{
  char *filename = new char [FILENAME_MAX];
  char *opath = new char [FILENAME_MAX];
  char *_opath = NULL;
  char *iseq  = new char [FILENAME_MAX];
  char *_iseq = NULL;
  char *windowTitle = new char[32];
  std::string username;

  // Configuration variable
  string baseName;
  string extension;
  int n0;
  
  ///////////////////////////////////////////////////////////////////////
  //
  // Set options and paths
  //
  ///////////////////////////////////////////////////////////////////////

  // Get the user login name
  vpIoTools::getUserName(username);
#ifdef WIN32
  sprintf(opath, "%s", getenv("TEMP"));
#else
  sprintf(opath, "/tmp/%s", username.c_str());
#endif
  if (vpIoTools::checkDirectory(opath) == false) {
    try {
      // Create the dirname
      cout << "Create: " << opath << endl;
      vpIoTools::makeDirectory(opath);
    }
    catch (...) {
      cerr << endl
	   << "ERROR:" << endl;
      cerr << "  Cannot create " << opath << " directory..." << endl;
      delete [] opath;
      delete [] filename;
      return -1;
    }
  }
  
#ifdef WIN32
  sprintf(opath, "%s/UsNeedleDetection", getenv("TEMP"));
#else
  sprintf(opath, "/tmp/%s/UsNeedleDetection", username.c_str());
#endif
  _opath = opath;
  
  vpParseArgv::vpArgvInfo argTable[] =
    {
      {NULL, vpParseArgv::ARGV_HELP, NULL, NULL,"     "},
      {NULL, vpParseArgv::ARGV_HELP, NULL, NULL,"2D US Needle Detection. \n"},
      {NULL, vpParseArgv::ARGV_HELP, NULL, NULL,"     "},
      {"-iseq", vpParseArgv::ARGV_STRING, (char *) 1, (char *) &_iseq,
       "Input sequence.\n"
       "         Path to the image sequence.  \n"},
      {"-opath", vpParseArgv::ARGV_STRING, (char *) 1, (char *) &_opath,
       "Output directory.\n"
       "         Directory which will contain the generated\n"
       "         data files.  Default value: "},
      {NULL, vpParseArgv::ARGV_HELP, NULL, NULL,"     "},
      {NULL, vpParseArgv::ARGV_END, NULL,NULL,NULL}
    } ;

  //Parsing of the table
  if (vpParseArgv::parse(&argc,argv,argTable,0)) {
    cout << "Usage : " << argv[0] << " [-iseq] [-opath directory] [-help] "
	 << endl;
    delete [] iseq;
    delete [] filename;
    exit(EXIT_SUCCESS);
  }

  if (_iseq!= NULL)
    sprintf(iseq, "%s", _iseq);
  else {
    cout << "Check your -iseq option." << endl;
    exit(EXIT_SUCCESS);
  }

  if (_opath != NULL)
    sprintf(opath, "%s", _opath);
  
  sprintf(filename, "%s", iseq);
  
  // Check input file
  vpIoTools::setBaseDir(string(filename));
  vpIoTools::setBaseName(string("sequence.txt"));

  if (vpIoTools::checkFilename(vpIoTools::getFullName())) {
	  cout << "Found configuration file: "
	       << vpIoTools::path(vpIoTools::getFullName()) << endl;
  }
  else {
    cout << "Error: Could not read configuration file " << vpIoTools::getFullName()
	 << endl;
    delete [] filename;
    delete [] iseq;
    delete [] opath;
    return 0;
  }

  // Read sequence configuration file
  if (vpIoTools::loadConfigFile(vpIoTools::getFullName())) {
    if (!vpIoTools::readConfigVar("baseName", baseName)) {
      cerr << "Error: Failed to load parameter baseName." << endl;
      exit(EXIT_FAILURE);
    }
    if (!vpIoTools::readConfigVar("extension", extension)) {
      cerr << "Error: Failed to load parameter extension." << endl;
      exit(EXIT_FAILURE);
    }
    if (!vpIoTools::readConfigVar("n0", n0)) {
      cerr << "Error: Failed to load parameter n0." << endl;
      exit(EXIT_FAILURE);
    }
  }
  else {
    cerr << "Error: Could not read configuration file " << filename << endl;
    delete [] filename;
    delete [] iseq;
    delete [] opath;
    exit(EXIT_FAILURE);
  }

  vpIoTools::setBaseName(string(baseName));

  // Check output directory
  if (vpIoTools::checkDirectory(opath) == false) {
    try {
      // Create the dirname
      cout << "Create: " << opath << endl;
      vpIoTools::makeDirectory(opath);
    }
    catch (...) {
      cerr << endl
	   << "ERROR:" << endl;
      cerr << "  Cannot create " << opath << " directory..." << endl;
      cout << "Check your -opath <path> option..." << endl << endl;
      delete [] opath;
      delete [] filename;
      return -1;
    }
  }
  
  ///////////////////////////////////////////////////////////////////////
  //
  // Initializations.
  //
  ///////////////////////////////////////////////////////////////////////
  
  // Load image parameters
  cout << "Base name: " << baseName << endl;
  cout << "Full name: " << vpIoTools::getFullName() << endl;
  cout << "Extension: " << extension << endl;

  sprintf(filename, "%05d.%s", n0, extension.c_str());

  vpIoTools::setBaseName(filename);

  if (!vpIoTools::checkFilename(vpIoTools::getFullName())) {
	  cout << "Error: Could not read file " << vpIoTools::getFullName() << endl;
    exit(EXIT_FAILURE);
  }

  // Read the first image
  vpImage<unsigned char> I;
  cout << "Reading " << vpIoTools::path(vpIoTools::getFullName()) << "..." << flush;
  try {
	  vpImageIo::readPNG(I, vpIoTools::path(vpIoTools::getFullName()));
  }
  catch (vpImageException e) {
	  cout << "Caught exception: " << e.getMessage() << endl;
	  exit(EXIT_FAILURE);
  }
  cout << "done." << endl;

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
  us2DNeedleModel needle(2);
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
  sprintf(filename, "%s/needle.dat", opath);
  std::ofstream ofile(filename);
  std::cout << "Results will be saved in " << filename << std::endl;

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

  sprintf(filename, "%05d.%s", n0++, extension.c_str());

  vpIoTools::setBaseName(filename);

  unsigned int it = 1;
  vpMatrix tipStd(2, 2);
  vpColVector tipMean;
  vpColVector evalue, evector;
  vpMatrix rendering;

  while (vpIoTools::checkFilename(vpIoTools::getFullName())) {
	  vpImageIo::read(I, vpIoTools::getFullName());
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

    // Ensure the matrix is symmetric
	/*
    tipStd[0][1] = tipStd[1][0];

    tipStd.eigenValues(evalue, evector);
    P.set_i(tipMean[0]);
    P.set_j(tipMean[1]);
    
    vpDisplay::displayEllipse(I, P, sqrt(evalue[0]), sqrt(evalue[1]),
			      atan2(evector.column(1)[0], evector.column(1)[1]),
			      false, vpColor::green);
    */
    vpDisplay::flush(I);


    //cin.ignore();

	sprintf(filename, "%05d.%s", n0++, extension.c_str());

	vpIoTools::setBaseName(filename);
    ++it;

  }

#if defined VISP_HAVE_DISPLAY
  delete display;
#endif
  ofile.close();
  exit(0);
  return 0;
}
