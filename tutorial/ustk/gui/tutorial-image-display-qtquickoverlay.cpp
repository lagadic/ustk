//! \example tutorial-image-display-qwidget.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_gui/usImageDisplayWidgetQmlOverlay.h>
#include <visp3/ustk_gui/usTracker2DQtWrapper.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{
  if (us::getDataSetPath().empty()) {
    std::cout << "You should fill USTK_DATASET_PATH to run this example\n";
    return 0;
  }

  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK tracking 2D demo"));

  usImagePreScan2D<unsigned char> preScan;
  usImageIo::read(preScan, us::getDataSetPath() + "/pre-scan/2D_xml/prescan2d.xml");

  usImageDisplayWidgetQmlOverlay *widget = new usImageDisplayWidgetQmlOverlay();
  widget->updateFrame(preScan);

  usTracker2DQtWrapper *tracker = new usTracker2DQtWrapper;
  QObject::connect(widget, SIGNAL(startTracking(vpRectOriented)), tracker, SLOT(initTracker(vpRectOriented)));
  QObject::connect(widget, SIGNAL(stopTracking()), tracker, SLOT(stopTracking()));

  // missing grabber connections

  QMainWindow window;
  window.setCentralWidget(widget);
  window.showMaximized();

  return app.exec();
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_template_tracking to run this tutorial" << std::endl;
  return 0;
}

#endif
