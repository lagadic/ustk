//! \example tutorial-confidence-map.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>
#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

int main(int argc, const char *argv[])
{
  std::string filename;

  // Command line options
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--input <filename>] [--help]" << std::endl;
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

  usImagePreScan2D<unsigned char> image, confidence;
  usImageIo::read(image, filename);

  usScanlineConfidence2D confidenceProcess;
  confidenceProcess.run(confidence, image);

  std::string outFileName = std::string("confidence.xml");
  std::cout << outFileName << std::endl;

  usImageIo::write(confidence, outFileName);

  return 0;
}
