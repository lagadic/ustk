//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) && defined(USTK_GRABBER_HAVE_QT5_WIDGETS) && defined(VISP_HAVE_X11)

#include <QThread>
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
  usDataGrabbed<usImagePreScan2D<unsigned char> > localFrame;

  //Prepare display
  vpDisplayX * displayX = NULL;
  bool displayInit = false;

  bool captureRunning = true;

  //qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->sendAcquisitionParameters();
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();


  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      //local copy for vpDisplay
      localFrame = *grabbedFrame;

      std::cout <<"MAIN THREAD received frame No : " << localFrame.getFrameCount() << std::endl;

      //init display
      if(!displayInit && localFrame.getHeight() !=0 && localFrame.getHeight() !=0) {
        displayX = new vpDisplayX(localFrame);
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(localFrame);
        vpDisplay::flush(localFrame);
        //vpTime::wait(10); // wait to simulate a local process running on last frame grabbed
      }
    }
    else {
      vpTime::wait(10);
      //std::cout << "wait new frame" << std::endl;
    }
  }while(captureRunning);


  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial" << std::endl;
  return 0;
}

#endif
