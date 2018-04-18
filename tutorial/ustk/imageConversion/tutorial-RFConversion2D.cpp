#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_FFTW

#include <visp3/core/vpTime.h>
#include <visp3/ustk_core/usRFToPostScan2DConverter.h>
#include <visp3/ustk_core/usImageIo.h>

int main(int argc, char **argv)
{
  std::string filename;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <RF2D.rf> | <RF2D.mhd>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (!env_ipath.empty())
      filename = env_ipath + "/RFElasto/image00010.mhd";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImageRF2D<short int> rfImage;

  usImagePreScan2D<unsigned char> prescanImage;

  // Warning : if you read .rf files, you have to fill the image settings by yourself after reading it.
  usImageIo::read(rfImage, filename);

  std::cout << "end reading" << std::endl;

  // scan-conversion
  usRFToPreScan2DConverter converter;

  double startTime = vpTime::measureTimeMs();
  std::cout << "init converter..." << std::endl;

  double endInitTime = vpTime::measureTimeMs();
  std::cout << "init time (sec) = " << (endInitTime - startTime) / 1000.0 << std::endl;

  std::cout << "converting..." << std::endl;
  converter.convert(rfImage, prescanImage);

  std::cout << prescanImage;

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - endInitTime) / 1000.0 << std::endl;

  std::cout << "writing pre-scan..." << std::endl;
  std::string outFileName = "prescan.xml";
  usImageIo::write(prescanImage, outFileName);

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
