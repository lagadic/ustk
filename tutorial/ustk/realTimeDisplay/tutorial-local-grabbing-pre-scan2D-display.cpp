//! \example tutorial-local-grabbing-pre-scan2D-display.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_GRABBER)

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>
#include <visp3/ustk_gui/usImageDisplayWidget.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{

  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK display widget"));

  // image
  usImagePreScan2D<unsigned char> *preScan = new usImagePreScan2D<unsigned char>(50, 50);

  // Qt widgets
  usImageDisplayWidget *widget = new usImageDisplayWidget();
  widget->updateFrame(*preScan);
  QMainWindow window;
  window.setCentralWidget(widget);
  window.show();

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  // qtGrabber->setVerbose(true);
  qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;    // 4DC7 id = 15
  header.slotId = 0;      // top slot id = 0
  header.imagingMode = 0; // B-mode = 0
  qtGrabber->initAcquisition(header);
  qtGrabber->runAcquisition();
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();
  // our grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      preScan = qtGrabber->acquire();
      widget->updateFrame(*preScan);
    }
  } while (window.isVisible());

  // disconnect from server
  qtGrabber->disconnectFromServer();
  grabbingThread->quit();
  return 0;
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_grabber to run this tutorial" << std::endl;
  return 0;
}

#endif
