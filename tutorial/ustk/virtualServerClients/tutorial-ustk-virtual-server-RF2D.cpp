//! \example tutorial-ustk-virtual-server-RF2D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) &&                                                           \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>
#include <visp3/ustk_io/usImageIo.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberRF2D *qtGrabber = new usNetworkGrabberRF2D();
  qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();

  // record option
  if (qApp->arguments().contains(QString("--output"))) {
    qtGrabber->activateRecording(
        qApp->arguments().at(qApp->arguments().indexOf(QString("--output")) + 1).toStdString());
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 0;      // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  usFrameGrabbedInfo<usImageRF2D<short int> > *grabbedFrame;

  // prepare converter
  usImagePreScan2D<unsigned char> preScanImage;
  usRFToPreScan2DConverter converter;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *display = new vpDisplayX();
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *display = new vpDisplayGDI();
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV *display = new vpDisplayOpenCV();
#endif

  bool captureRunning = true;
  bool displayInit = false;
  // qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);
  std::cout << "init success" << std::endl;
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      converter.convert(*grabbedFrame, preScanImage);

      if (!displayInit) {
        display->init(preScanImage);
        displayInit = true;
      }
      vpDisplay::display(preScanImage);
      vpDisplay::flush(preScanImage);
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  delete display;

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X  to run this tutorial"
            << std::endl;
  return 0;
}

#endif
