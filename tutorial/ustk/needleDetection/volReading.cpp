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
#include <visp3/ustk_io/usSequenceReader3D.h>
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

  std::string vol_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      vol_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <file.vol>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (vol_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      vol_filename = env_ipath + "/vol/01.vol";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImagePreScan3D<unsigned char> image;

  //usImageIo::read(image, vol_filename);
  usSequenceReader3D<usImagePreScan3D<unsigned char> > reader;
  reader.setSequenceFileName(vol_filename);

  vpImage<unsigned char> firstFrame;
  char buffer[300];

  std::string base = "vol%d-frame%d.png";

  //reading 10th volume with getVolume method, and writing the 1st frame to check it
  reader.getVolume(image,10);
  firstFrame.resize(image.getBModeSampleNumber(), image.getScanLineNumber());
  for (unsigned int i = 0; i < image.getBModeSampleNumber(); i++) {
    for (unsigned int j = 0; j < image.getScanLineNumber(); j++) {
      firstFrame(i, j, image(j, i, 0));
    }
  }
  sprintf(buffer, base.c_str(), 10, 0);
  vpImageIo::write(firstFrame, buffer);

  //Reading all the sequence with the  reader
  /*
  while(!reader.end()) {
    reader.acquire(image);
    firstFrame.resize(image.getBModeSampleNumber(), image.getScanLineNumber());
    for (unsigned int k = 0; k < image.getDimZ(); k++) {
      for (unsigned int i = 0; i < image.getBModeSampleNumber(); i++) {
        for (unsigned int j = 0; j < image.getScanLineNumber(); j++) {
          firstFrame(i, j, image(j, i, k));
        }
      }
      sprintf(buffer, base.c_str(), reader.getImageNumber(), k);
      vpImageIo::write(firstFrame, buffer);
    }
  }*/
  return 0;
}
