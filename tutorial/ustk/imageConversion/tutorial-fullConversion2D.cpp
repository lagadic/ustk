#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_FFTW

#include <visp3/ustk_core/usRFToPostScan2DConverter.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/core/vpTime.h>

int main(int argc, char** argv)
{
  std::string filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <RF2D.rf>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      filename = env_ipath + "/rf/signal.rf";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImageRF2D<short int> rfImage;

  usImagePostScan2D<unsigned char> postscanImage;
  postscanImage.setHeightResolution(0.0005);
  postscanImage.setWidthResolution(0.0005);

  usImageIo::read(rfImage,filename);

  //settings used for signal.rf file in ustk-dataset
  rfImage.setScanLinePitch(0.010625);
  rfImage.setTransducerRadius(0.0398);
  rfImage.setDepth(0.15);

  postscanImage.setTransducerSettings(rfImage);

  std::cout << "end reading" << std::endl;

  //scan-conversion
  usRFToPostScan2DConverter converter;

  double startTime = vpTime::measureTimeMs();
  std::cout << "init converter..." << std::endl;

  converter.setConversionParameters(postscanImage,rfImage.getRFSampleNumber()/10,rfImage.getScanLineNumber(),10);

  double endInitTime = vpTime::measureTimeMs();
  std::cout << "init time (sec) = " << (endInitTime - startTime) / 1000.0 << std::endl;

  std::cout << "converting..." << std::endl;
  converter.convert(rfImage,postscanImage);

  std::cout << postscanImage;

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - endInitTime) / 1000.0 << std::endl;

  std::cout << "writing post-scan..." << std::endl;
  std::string outFileName ="postscan.xml";
  usImageIo::write(postscanImage,outFileName);

  return 0;
}

#else
#include <iostream>
int main()
{
  std::cout << "You should install FFTW library to run this tutorial" << std::endl;
  return 0;
}

#endif
