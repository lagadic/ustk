#include <visp3/core/vpTime.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usSequenceReader.h>

int main(int argc, char **argv)
{
  std::string filename;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <preScan3D.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (!env_ipath.empty())
      filename = env_ipath + "/pre-scan/2D_xml/prescan2d.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  usImagePreScan2D<unsigned char> prescanImage;
  prescanImage.resize(128, 480, 16);
  usImagePostScan2D<unsigned char> postscanImage;

  usImageIo::read(prescanImage, filename);

  // scan-conversion
  usPreScanToPostScan2DConverter converter;

  converter.convert(prescanImage, postscanImage);

  std::cout << "converted image : " << std::endl;
  std::cout << postscanImage;

  std::cout << "writing post-scan..." << std::endl;
  std::string outFileName = "postscan.xml";
  usImageIo::write(postscanImage, outFileName);

  return 0;
}
