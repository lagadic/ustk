//! \example tutorial-image-display-qtquickoverlay.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>
#include <visp3/ustk_gui/usImageDisplayWidgetQmlOverlay.h>
#include <visp3/ustk_gui/usTracker2DQtWrapper.h>
#include <visp3/ustk_gui/usUltrasonixClientWidget.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK tracking 2D demo"));

  usImageDisplayWidgetQmlOverlay *widget = new usImageDisplayWidgetQmlOverlay();

  usUltrasonixClientWidget *ultrasonixControlWidet = new usUltrasonixClientWidget();

  QWidget *centralWidget = new QWidget();
  QHBoxLayout *mainLayout = new QHBoxLayout();
  mainLayout->addWidget(ultrasonixControlWidet);
  mainLayout->addWidget(widget, 3);
  // mainLayout->addWidget(robotControlPanel);
  centralWidget->setLayout(mainLayout);

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  usTracker2DQtWrapper *tracker = new usTracker2DQtWrapper();

  // controls to start / stop tracking
  QObject::connect(widget, SIGNAL(startTracking(vpRectOriented)), tracker, SLOT(initTracker(vpRectOriented)));
  QObject::connect(widget, SIGNAL(stopTracking()), tracker, SLOT(stopTracking()));

  // grabber control
  qRegisterMetaType<QHostAddress>("QHostAddress");
  qRegisterMetaType<usNetworkGrabber::usInitHeaderSent>("usNetworkGrabber::usInitHeaderSent");
  QObject::connect(ultrasonixControlWidet, SIGNAL(connectToServer(QHostAddress)), qtGrabber,
                   SLOT(connectToServer(QHostAddress)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(initAcquisition(usNetworkGrabber::usInitHeaderSent)), qtGrabber,
                   SLOT(initAcquisitionSlot(usNetworkGrabber::usInitHeaderSent)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(center3DProbeMotor()), qtGrabber, SLOT(center3DProbeMotor()));
  QObject::connect(ultrasonixControlWidet, SIGNAL(runAcquisition()), qtGrabber, SLOT(runAcquisition()));
  QObject::connect(ultrasonixControlWidet, SIGNAL(stopAcquisition()), qtGrabber, SLOT(stopAcquisition()));

  // send new images via qt signal
  qRegisterMetaType<usImagePostScan2D<unsigned char> >("usImagePreScan2D<unsigned char>");
  QObject::connect(qtGrabber, SIGNAL(newFrame(usImagePreScan2D<unsigned char>)), widget,
                   SLOT(updateFrame(usImagePreScan2D<unsigned char>)));
  QObject::connect(qtGrabber, SIGNAL(newFrame(usImagePreScan2D<unsigned char>)), tracker,
                   SLOT(updateImage(usImagePreScan2D<unsigned char>)));

  // updates the GUI based on the tracking output
  QObject::connect(tracker, SIGNAL(newTrackedRectangle(vpRectOriented)), widget,
                   SLOT(updateRectPosition(vpRectOriented)));

  widget->updateFrame(vpImage<unsigned char>(200, 200));

  QMainWindow window;
  window.setCentralWidget(centralWidget);
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
