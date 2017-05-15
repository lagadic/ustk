#include <visp3/ustk_core/usScanConverter2D.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/core/vpTime.h>

int main(int argc, char** argv)
{
  std::string filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <preScan3D.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      filename = env_ipath + "/pre-scan/2D_xml/prescan2d.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImagePreScan2D<unsigned char> prescanImage;
  prescanImage.resize(128,480,16);
  usImagePostScan2D<unsigned char> postscanImage;
  postscanImage.setHeightResolution(0.0005);
  postscanImage.setWidthResolution(0.0005);

  usImageIo::read(prescanImage,filename);

  postscanImage.setTransducerSettings(prescanImage);


  std::cout << "end reading" << std::endl;

  //scan-conversion
  usScanConverter2D converter;

  double startTime = vpTime::measureTimeMs();
  std::cout << "init converter..." << std::endl;

  converter.init(postscanImage,480,128);

  double endInitTime = vpTime::measureTimeMs();
  std::cout << "init time (sec) = " << (endInitTime - startTime) / 1000.0 << std::endl;

  std::cout << "converting..." << std::endl;
  converter.run(prescanImage,postscanImage);

  std::cout << postscanImage;

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - endInitTime) / 1000.0 << std::endl;

  std::cout << "writing post-scan..." << std::endl;
  std::string outFileName ="postscan.xml";
  usImageIo::write(postscanImage,outFileName);

  return 0;
}
