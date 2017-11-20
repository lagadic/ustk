//! \example tutorial-ultrasonix-qt-grabbing-post-scan-bi-plan.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <QApplication>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberPostScanBiPlan.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberPostScanBiPlan *qtGrabber = new usNetworkGrabberPostScanBiPlan();
  qtGrabber->connectToServer();
  // qtGrabber->setVerbose(true);
  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 14;     // bi-plan probe id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 23; // bi-plan = 23

  std::vector<usFrameGrabbedInfo<usImagePostScan2D<unsigned char> > *> grabbedFrame;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *display1 = NULL;
  vpDisplayX *display2 = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *display1 = NULL;
  vpDisplayGDI *display2 = NULL;
#endif
  bool displayInit = false;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // grab a 640*480 post-scan image
  qtGrabber->setPostScanMode(true);
  qtGrabber->setPostScanHeigh(300);
  qtGrabber->setPostScanWidth(300);

  qtGrabber->sendAcquisitionParameters();
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {

      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame[0]->getFrameCount() << " and "
                << grabbedFrame[1]->getFrameCount() << std::endl;

      // init display
      if (!displayInit && grabbedFrame[0]->getHeight() != 0 && grabbedFrame[0]->getWidth() != 0) {
#if defined(VISP_HAVE_X11)
        display1 = new vpDisplayX(*(grabbedFrame[0]));
        display2 = new vpDisplayX(*(grabbedFrame[1]));
#elif defined(VISP_HAVE_GDI)
        display1 = new vpDisplayGDI(*(grabbedFrame[0]));
        display2 = new vpDisplayGDI(*(grabbedFrame[1]));
#endif
        displayInit = true;
      }

      // processing display
      if (displayInit) {
        vpDisplay::display(*(grabbedFrame[0]));
        vpDisplay::display(*(grabbedFrame[1]));
        vpDisplay::flush(*(grabbedFrame[0]));
        vpDisplay::flush(*(grabbedFrame[1]));
      }
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  if (displayInit) {
    delete display1;
    delete display2;
  }

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
