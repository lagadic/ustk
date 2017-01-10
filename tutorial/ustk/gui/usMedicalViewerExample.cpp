#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <visp3/ustk_gui/usMedicalImageViewer.h>

int main( int argc, char** argv )
{
  // QT application
  QApplication app( argc, argv );

  //std::string fileName = "/home/mpouliqu/Documents/ustk-dataset/3D/volumeTest.mhd";
  std::string fileName = "/home/mpouliqu/Documents/ustk-dataset/pre-scan/3D_mhd/volume.mhd";
  usMedicalImageViewer medicalImageViewer(fileName);
  medicalImageViewer.show();

  return app.exec();
}
