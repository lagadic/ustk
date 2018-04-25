//! \example tutorial-ustk-virtual-server-postScan2D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) &&                                                           \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberPostScan2D.h>
#include <visp3/ustk_core/usImageIo.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberPostScan2D *qtGrabber = new usNetworkGrabberPostScan2D();
  qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();

  // record option
  if (qApp->arguments().contains(QString("--output"))) {
    qtGrabber->activateRecording(
        qApp->arguments().at(qApp->arguments().indexOf(QString("--output")) + 1).toStdString());
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 0;     // 4DC7 id = 15
  header.slotId = 0;      // top slot id = 0
  header.imagingMode = 0; // B-mode = 0

  usFrameGrabbedInfo<usImagePostScan2D<unsigned char> > *grabbedFrame;

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
      QString filename = QString("img") + QString::number(grabbedFrame->getFrameCount()) + QString(".xml");
      usImageIo::write(*grabbedFrame, filename.toStdString());

      std::cout << *grabbedFrame << std::endl;

      if (!displayInit) {
        display->init(*grabbedFrame);
        qtGrabber->useVpDisplay(display);
        displayInit = true;
      }
      vpDisplay::display(*grabbedFrame);
      vpDisplay::flush(*grabbedFrame);
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
