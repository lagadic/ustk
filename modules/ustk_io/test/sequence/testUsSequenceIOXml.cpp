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
#if defined(VISP_HAVE_XML2)

#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/ustk_io/usSequenceWriter.h>

#include <string>
#include <vector>
#include <time.h>

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



/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */

int main(int argc, const char** argv)
{
  try {
    std::string opt_opath;
    std::string opath;
    std::string filename;
    std::string username;

    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout <<  "  testUSSequenceIOXml.cpp" <<std::endl << std::endl ;
    std::cout <<  "  writing and reading ultrasound sequences using usSequenceReader and usSequenceWriter" << std::endl ;
    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;

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
      opath = opt_opath;

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
        exit(-1);
      }
    }

    filename = dirname + vpIoTools::path("/") + "sequenceRF2D.xml";

    //Init values in reference image
    usImageRF2D<unsigned char> rf2DReference;
    //init image
    vpImage<unsigned char> image;
    image.resize(320,128,123);
    //init settings
    rf2DReference.setAxialResolution(0.0005);
    rf2DReference.setScanLinePitch(0.0045);
    rf2DReference.setTransducerRadius(0.05478);
    rf2DReference.setTransducerConvexity(true);
    rf2DReference.setData(image);

    std::vector<usImageRF2D<unsigned char> > ImageBufferRef;
    for(int i=0;i<4;i++) {
      ImageBufferRef.push_back(rf2DReference);
    }

    usSequenceWriter<usImageRF2D<unsigned char> > writer;
    writer.setSequenceFileName(filename);
    writer.setImageFileName(std::string("mysubdir/sequenceRF2D%04d.png"));
    writer.setFrameRate(15);
    int i=0;
    while( i<4) {
      writer.saveImage(ImageBufferRef.at(i));
      i++;
    }
    std::cout << "read i : " << i <<std::endl;
    writer.close();

    std::cout << "Written in " << filename << std::endl;
    std::cout << rf2DReference;
    std::cout << "height resolution : " << rf2DReference.getAxialResolution() << std::endl;

    //read the image we just wrote    
    usSequenceReader<usImageRF2D<unsigned char> > reader;

    std::vector<usImageRF2D<unsigned char> > ImageBuffer;

    reader.setSequenceFileName(filename);
    i = 0;
    while(!reader.end()) {
      usImageRF2D<unsigned char> rf2D;
      reader.acquire(rf2D);
      ImageBuffer.push_back(rf2D);
      i++;
    }

    std::cout << "Read from " << filename << std::endl ;
    std::cout << ImageBuffer.at(0);
    std::cout << "height resolution : " << ImageBuffer.at(0).getAxialResolution() << std::endl;

    bool testPassed = true;

    if(reader.getFrameRate() != writer.getFrameRate() || ImageBuffer.size() != ImageBufferRef.size()){
      testPassed = false;
      std::cout << "reader framerate : " << reader.getFrameRate() << std::endl;
      std::cout << "writer framerate : " << writer.getFrameRate() << std::endl;
      std::cout << "size buff : " << ImageBuffer.size() << std::endl;
      std::cout << "size buff ref :  " << ImageBufferRef.size() << std::endl;
    }

    //checking every image
    for(unsigned int i = 0; i < ImageBuffer.size();i++) {
      if(ImageBuffer.at(i) != ImageBufferRef.at(i) ) {
        testPassed = false;
        std::cout << "images not equal at index : " << i << std::endl;
      }
    }

    //-----------------Testing loop cycling-----------------

    //read the image we just wrote
    usSequenceReader<usImageRF2D<unsigned char> > readerCycling;

    //std::vector<usImageRF2D<unsigned char> > ImageBufferCycling;

    //ref timer
    time_t refTimer;
    time(&refTimer);

    readerCycling.setSequenceFileName(filename);
    readerCycling.setLoopCycling(true);
    i = 0;
    while(!readerCycling.end()) {
      usImageRF2D<unsigned char> rf2D;
      readerCycling.acquire(rf2D);
      time_t testTimer;
      time(&testTimer);
      double seconds = difftime(testTimer,refTimer);
      if(seconds>1) //after 1 second we stop the cycle
        readerCycling.setLoopCycling(false);
      i++;
    }

    //------------------------------------------------------

    // Clean up memory allocated by the xml library
    vpXmlParser::cleanup();
    std::cout << "Test exit code : " << (int)!testPassed << std::endl;
    return !testPassed;
  }
  catch(const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return 1;
  }
}

#else

int main()
{
  std::cout << "Xml parser requires libxml2." << std::endl;
  return 0;
}
#endif
