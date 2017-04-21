//! \example tutorial-ultrasonix-qt-grabbing-post-scan.cpp

#include <iostream>
#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) && defined(USTK_GRABBER_HAVE_QT5_WIDGETS)

#include <QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPostScan.h>

#include <visp3/gui/vpDisplayX.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPostScan * qtGrabber = new usNetworkGrabberPostScan();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.transmitFrequency = 4000000;
  header.samplingFrequency = 2500000;
  header.imagingMode = 0; //B-mode = 0
  header.postScanMode = true;
  header.postScanHeigh = 480;
  header.postScanWidth = 640;
  header.imageDepth = 140; //in mm
  header.sector = 100; //in %

  // 3D acquisition
  header.activateMotor = true;
  header.framesPerVolume = 3;
  header.degreesPerFrame = 35;

  //prepare image;
  usDataGrabbed<usImagePostScan2D<unsigned char> >* grabbedFrame;

  //Prepare display
  vpDisplayX * displayX = NULL;
  bool displayInit = false;

  bool captureRunning = true;

  //qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << *grabbedFrame << std::endl;

      if(grabbedFrame->getFrameCount()%header.framesPerVolume == 0) {
        std::cout << "total vol ,frame " << grabbedFrame->getFrameCount() << std::endl;
      }

      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getHeight() !=0) {
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
  }while(captureRunning);


  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X  to run this tutorial" << std::endl;
  return 0;
}

#endif
