//! \example tutorial-image-display-qwidget.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_gui/usImageDisplayWidget.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{
  if (us::getDataSetPath().empty()) {
    std::cout << "You should fill USTK_DATASET_PATH to run this example\n";
    return 0;
  }

  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK elastography demo"));

  usImagePreScan2D<unsigned char> preScan;
  usImageIo::read(preScan, us::getDataSetPath() + "/pre-scan/2D_xml/prescan2d.xml");

  usImageDisplayWidget *widget = new usImageDisplayWidget();
  widget->updateFrame(preScan);

  QMainWindow window;
  window.setCentralWidget(widget);
  window.showMaximized();

  return app.exec();
}

#else
int main()
{
  std::cout << "You should build ustk_gui to run this tutorial" << std::endl;
  return 0;
}

#endif
