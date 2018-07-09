//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->connectToServer();

  // record option
  if (qApp->arguments().contains(QString("--record"))) {
    qtGrabber->activateRecording(
        qApp->arguments().at(qApp->arguments().indexOf(QString("--record")) + 1).toStdString());
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  if (qApp->arguments().contains(QString("--probeID"))) {
        header.probeId = qApp->arguments().at(qApp->arguments().indexOf(QString("--probeID")) + 1).toInt();
  }
  else
    header.probeId = 15;    // 4DC7 id = 15 by default

  if (qApp->arguments().contains(QString("--slotID"))) {
    header.slotId = qApp->arguments().at(qApp->arguments().indexOf(QString("--slotID")) + 1).toInt();
  }
  else
    header.slotId = 0;      // top slot id = 0 by default

  if (qApp->arguments().contains(QString("--imagingMode"))) {
        header.imagingMode = qApp->arguments().at(qApp->arguments().indexOf(QString("--imagingMode")) + 1).toInt();
  }
  else
    header.imagingMode = 0; // B-mode = 0 by default


  // prepare image;
  usFrameGrabbedInfo<usImagePreScan2D<unsigned char> > *grabbedFrame;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *display = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *display = NULL;
#endif
  bool displayInit = false;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->setMotorPosition(37);
  qtGrabber->sendAcquisitionParameters();

  // Send the command to run the acquisition
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  // our grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      // init display
      if (!displayInit && grabbedFrame->getHeight() != 0 && grabbedFrame->getWidth() != 0) {
#if defined(VISP_HAVE_X11)
        display = new vpDisplayX(*grabbedFrame);
#elif defined(VISP_HAVE_GDI)
        display = new vpDisplayGDI(*grabbedFrame);
#endif
        qtGrabber->useVpDisplay(display);
        displayInit = true;
      }

      // processing display
      if (displayInit) {
        if(vpDisplay::getClick(*grabbedFrame, false))
          captureRunning = false;
        vpDisplay::display(*grabbedFrame);
        vpDisplay::displayText(*grabbedFrame,20,20,std::string("Click to exit..."),vpColor::red);
        vpDisplay::flush(*grabbedFrame);
      }
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  qtGrabber->stopAcquisition();

  if (displayInit) {
    delete display;
  }

  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial"
            << std::endl;
  return 0;
}

#endif
