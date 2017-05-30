//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>

#include <visp3/gui/vpDisplayX.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan * qtGrabber = new usNetworkGrabberPreScan();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  //prepare image;
  usDataGrabbed<usImagePreScan2D<unsigned char> >* grabbedFrame;

  //Prepare display
  vpDisplayX * displayX = NULL;
  bool displayInit = false;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // Send the command to run the acquisition
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  //our grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getWidth() !=0) {
        displayX = new vpDisplayX(*grabbedFrame);
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(*grabbedFrame);
        vpDisplay::flush(*grabbedFrame);
      }
    }
    else {
      vpTime::wait(10);
    }
  } while(captureRunning);

  if(displayInit) {
    delete displayX;
  }

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial" << std::endl;
  return 0;
}

#endif
