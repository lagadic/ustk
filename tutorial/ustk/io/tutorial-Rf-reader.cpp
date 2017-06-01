#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usRfReader.h>

int main(int argc, char** argv)
{
  std::string rf_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      rf_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <rfData.rf>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (rf_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      rf_filename = env_ipath + "/rf/signal.rf";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }
  //read us image
  usImageRF2D<short int> rfImage;
  usImageIo::read(rfImage,rf_filename);

  std::cout << rfImage;

  usImagePreScan2D<unsigned char> preScanImage;
  usRFToPreScan2DConverter converter;
  converter.convert(rfImage,preScanImage);

  usImageIo::write(preScanImage,"test.xml");

  //wait until user closes the window
  return 0;
}
#else
#include <iostream>
int main()
{
  std::cout << "You should install libfftw to run the Rf to pre-scan conversion" << std::endl;
}
#endif
