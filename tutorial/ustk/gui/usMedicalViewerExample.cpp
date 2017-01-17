#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_core/us.h>
#include <visp3/ustk_gui/usMedicalImageViewer.h>

int main(int argc, char** argv)
{
  std::string mhd_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      mhd_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <3D volume.mhd>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (mhd_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      mhd_filename = env_ipath + "/pre-scan/3D_mhd/volume.mhd";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  // QT application
  QApplication app( argc, argv );

  usMedicalImageViewer medicalImageViewer(mhd_filename);
  medicalImageViewer.show();

  return app.exec();
}
#else
#include <iostream>

int main()
{
  std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl;
}

#endif
