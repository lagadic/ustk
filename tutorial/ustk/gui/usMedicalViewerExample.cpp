#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usMedicalImageViewer.h>

int main( int argc, char** argv )
{
  // QT application
  QApplication app( argc, argv );

  //std::string fileName = "/home/mpouliqu/Documents/ustk-dataset/3D/volumeTest.mhd";
  std::string fileName = "/home/mpouliqu/Documents/ustk-dataset/pre-scan/3D_mhd/volume.mhd";
  //std::string fileName = "/udd/mpouliqu/soft/ustk/ustk-dataset/pre-scan/3D_mhd/volume.mhd";
  usMedicalImageViewer medicalImageViewer(fileName);
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
