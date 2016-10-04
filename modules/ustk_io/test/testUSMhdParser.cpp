/****************************************************************************
 *
 * Author:
 * Marc Pouliquen
 *
 *****************************************************************************/


/*!
  \example testUSMhdParser.cpp

  USTK MHD parser example.
  
  This example contains the declaration of a class used to read and write data
  in a mhd file like:
  \code
NDims = 3
DimSize = 186 233 163
ElementType = MET_UCHAR
ElementSpacing = 1 1 1
ElementByteOrderMSB = False
ElementDataFile = postscan3d.raw
UltrasoundImageType = POSTSCAN_3D
  \endcode
  
*/


#include <visp3/core/vpConfig.h>

#include <iostream>

#include <visp3/core/vpXmlParser.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/ustk_io/usMetaHeaderParser.h>

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
              -o <output data path>                               %s\n\
              Set data output path.\n\
              From this directory, creates the \"%s\"\n\
              subdirectory depending on the username, where \n\
              prescan2D.xml file is written.\n\
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
    std::string ipath;
    std::string opath;
    std::string filename;
    std::string username;

    usMetaHeaderParser testReferenceSettings;

    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout <<  "  testUSMhdParser.cpp" <<std::endl << std::endl ;
    std::cout <<  "  writing and reading ultrasound data using a the US mhd parser" << std::endl ;
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
    filename = opath + vpIoTools::path("/") + "postscan3d";

    //Init values in reference parser (same values in file read in test)
    usImagePostScan3D<unsigned char> postscan3DReference;
    postscan3DReference.resize(186,233,163);
    postscan3DReference.setElementSpacingX(1);
    postscan3DReference.setElementSpacingY(1);
    postscan3DReference.setElementSpacingZ(1);
    postscan3DReference.setScanLinePitch(0.0145);
    postscan3DReference.setProbeRadius(0.554);
    postscan3DReference.setProbeConvex(true);
    postscan3DReference.setFramePitch(0.258);
    postscan3DReference.setMotorRadius(0.025);
    postscan3DReference.setMotorConvex(true);
    postscan3DReference.setWidthResolution(0.0058);
    postscan3DReference.setHeightResolution(0.0058);

    std::cout << "Read from " << filename << std::endl ;
    std::cout << "Dim X : " << postscan3DReference.getANumber() << std::endl;
    std::cout << "Dim Y : " << postscan3DReference.getLineNumber() << std::endl;
    std::cout << "Dim Z: " << postscan3DReference.getFrameNumber() << std::endl;
    std::cout << "Spacing X: " << postscan3DReference.getElementSpacingX() << std::endl;
    std::cout << "Spacing Y: " << postscan3DReference.getElementSpacingY() << std::endl;
    std::cout << "Spacing Z: " << postscan3DReference.getElementSpacingZ() << std::endl;
    std::cout << "Scanline pitch : " << postscan3DReference.getScanLinePitch() << std::endl;
    std::cout << "Probe Radius : " << postscan3DReference.getProbeRadius() << std::endl;
    std::cout << "Frame pitch : " << postscan3DReference.getFramePitch() << std::endl;
    std::cout << "Motor Radius : " << postscan3DReference.getMotorRadius() << std::endl;
    std::cout << "Width resolution : " << postscan3DReference.getWidthResolution() << std::endl;
    std::cout << "Height resolution : " << postscan3DReference.getHeightResolution() << std::endl;

    //write image
    usImageIo::write(postscan3DReference,filename);

    //read the image we just wrote
    usImagePostScan3D<unsigned char> postscan3D;
    filename = opath + vpIoTools::path("/") + "postscan3d.mhd";
    usImageIo::read(postscan3D,filename);

    std::cout << "Read from " << filename << std::endl ;
    std::cout << "Dim X : " << postscan3D.getANumber() << std::endl;
    std::cout << "Dim Y : " << postscan3D.getLineNumber() << std::endl;
    std::cout << "Dim Z: " << postscan3D.getFrameNumber() << std::endl;
    std::cout << "Spacing X: " << postscan3D.getElementSpacingX() << std::endl;
    std::cout << "Spacing Y: " << postscan3D.getElementSpacingY() << std::endl;
    std::cout << "Spacing Z: " << postscan3D.getElementSpacingZ() << std::endl;
    std::cout << "Scanline pitch : " << postscan3D.getScanLinePitch() << std::endl;
    std::cout << "Probe Radius : " << postscan3D.getProbeRadius() << std::endl;
    std::cout << "Frame pitch : " << postscan3D.getFramePitch() << std::endl;
    std::cout << "Motor Radius : " << postscan3D.getMotorRadius() << std::endl;
    std::cout << "Width resolution : " << postscan3D.getWidthResolution() << std::endl;
    std::cout << "Height resolution : " << postscan3D.getHeightResolution() << std::endl;

    if(postscan3D==postscan3DReference) {
      std::cout << "Test passed !" << std::endl;
      return 0;
    }

    // Clean up memory allocated by the xml library
    vpXmlParser::cleanup();
    std::cout << "Test failed !" << std::endl;
    return 1;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
