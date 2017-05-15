#include <visp3/ustk_core/usConfig.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usDataFrameReader.h>

int main(int argc, char** argv)
{
  std::string mhd_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      mhd_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <rfData.rf>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (mhd_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      mhd_filename = env_ipath + "/post-scan/3D_mhd/volume.mhd";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }
  //read us image
  usImageRF2D<short int> rfImage;
  usDataFrameReader<usImageRF2D<short int> > reader;

  reader.setFileName("/home/mpouliqu/Documents/usData/rf/2014-11-26/frame001.rf");
  reader.acquire(rfImage);

  std::cout << rfImage;

  usImagePreScan2D<unsigned char> preScanImage;
  usRFToPreScan2DConverter converter;
  converter.convert(rfImage,preScanImage);

  usImageIo::write(preScanImage,"test.xml");

  //wait until user closes the window
  return 0;
}
