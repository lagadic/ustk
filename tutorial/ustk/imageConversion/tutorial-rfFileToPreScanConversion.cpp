#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_FFTW

#include <visp3/core/vpTime.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>
#include <visp3/ustk_io/usImageIo.h>

int main()
{
  std::string filename;

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  std::string env_ipath = us::getDataSetPath();
  if (!env_ipath.empty())
    filename = env_ipath + "/rf/signal.rf";
  else {
    std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
    return 0;
  }
  std::cout << filename << std::endl;
  usImageRF2D<short int> rfImage;
  usImagePreScan2D<unsigned char> prescanImage;

  usImageIo::read(rfImage, filename);

  // settings used for rf file in ustk-dataset
  rfImage.setScanLinePitch(0.010625);
  rfImage.setTransducerRadius(0.0398);
  rfImage.setDepth(0.15);

  std::cout << "end reading" << std::endl;

  // scan-conversion
  usRFToPreScan2DConverter converter;

  double startTime = vpTime::measureTimeMs();

  std::cout << "converting..." << std::endl;

  converter.convert(rfImage, prescanImage);

  std::cout << prescanImage;

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - startTime) / 1000.0 << std::endl;

  std::cout << "writing pre-scan..." << std::endl;
  std::string outFileName = "preScan.png";
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
