#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <visp3/ustk_gui/usMedicalImageViewer.h>

int main( int argc, char** argv )
{
  // QT application
  QApplication app( argc, argv );

  std::string fileName = "/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd";
  usMedicalImageViewer medicalImageViewer(fileName);
  medicalImageViewer.show();

  return app.exec();
}
