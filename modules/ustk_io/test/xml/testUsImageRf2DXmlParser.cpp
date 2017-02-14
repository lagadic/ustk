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

/*!
  \example testUsImageRf2DXmlParser.cpp

  USTK XML parser example.
  
  This example contains the declaration of a class used to read and write data
  in a xml file like:
  \code
<?xml version="1.0"?>
<config>
  <image_type>rf</image_type>
  <axial_resolution>0.0005</axial_resolution>
  <scanline_pitch>0.0045</scanline_pitch>
  <probe_radius>0.0547896</probe_radius>
  <is_convex>1</is_convex>
  <image_file_name>rf2D.png</image_file_name>
</config>
  \endcode
  
*/


#include <visp3/core/vpConfig.h>

#include <iostream>
#if defined(VISP_HAVE_XML2)

#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/ustk_io/usImageIo.h>

#include <string>

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
Write and read data in a xml file.\n\
\n\
SYNOPSIS\n\
  %s [-o <output image path>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -o <output data path>                                %s\n\
    Set data output path.\n\
    From this directory, creates the \"%s\"\n\
    subdirectory depending on the username, where \n\
    rf2D.xml file is written.\n\
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
    std::cout <<  "  testUsImageRf2DXmlParser.cpp" <<std::endl << std::endl ;
    std::cout <<  "  writing and reading ultrasound data using a the US xml parser" << std::endl ;
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

    filename = dirname + vpIoTools::path("/") + "rf2D.xml";

    //Init values in reference parser (same values in file read in test)
    vpImage<unsigned char> data(320,128, 128); // Set pixel intensity to 128

    usImageRF2D<unsigned char> rf2DReference;
    rf2DReference.setData(data);
    rf2DReference.setAxialResolution(0.0005);
    rf2DReference.setScanLinePitch(0.0045);
    rf2DReference.setTransducerRadius(0.05478);
    rf2DReference.setTransducerConvexity(true);

    usImageIo::write(rf2DReference,filename);

    std::cout << "Written in " << filename << std::endl;
    std::cout << rf2DReference;

    //read the image we just wrote
    usImageRF2D<unsigned char> rf2D;
    filename = dirname + vpIoTools::path("/") + "rf2D.xml";
    usImageIo::read(rf2D,filename);


    std::cout << "Read from " << filename << std::endl ;
    std::cout << rf2D;

    if(rf2D == rf2DReference) {
      std::cout << "Test passed !" << std::endl;
      return 0;
    }


    // Clean up memory allocated by the xml library
    vpXmlParser::cleanup();
    std::cout << "Test failed !" << std::endl;
    return 1;
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
