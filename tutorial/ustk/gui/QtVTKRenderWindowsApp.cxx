#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <visp3/ustk_gui/QtVTKRenderWindows.h>

int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );

  std::string fileName = "/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd";
  QtVTKRenderWindows myQtVTKRenderWindows(fileName);
  myQtVTKRenderWindows.show();

  return app.exec();
}
