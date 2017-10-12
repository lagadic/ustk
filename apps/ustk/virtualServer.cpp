#include "usVirtualServer.h"
#include <QApplication>

int main(int argc, char** argv)
{
  std::string filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <mysequence.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      filename = env_ipath + "/pre-scan/timestampSequence/sequenceTimestamps.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  QApplication app(argc, argv);

  usVirtualServer server(filename);




  //wait until user closes the window
  return app.exec();
}
