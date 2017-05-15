//! \example tutorial-ultrasonix-qt-grabbing-pre-scan-confidence.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_VIPER850)


#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

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
  usImagePreScan2D<unsigned char> confidence;

  //Prepare display
  vpDisplayX * displayX = NULL;
  vpDisplayX * displayXConf = NULL;
  bool displayInit = false;

  //prepare confidence
  usScanlineConfidence2D confidenceProcessor;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();


  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();
      confidenceProcessor.run(confidence,*grabbedFrame);
      //local copy for vpDisplay
      //localFrame = *grabbedFrame;

      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      std::cout << *grabbedFrame << std::endl;

      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getHeight() !=0) {
        displayX = new vpDisplayX(*grabbedFrame);
        displayXConf = new vpDisplayX(confidence);
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(*grabbedFrame);
        vpDisplay::flush(*grabbedFrame);
        vpDisplay::display(confidence);
        vpDisplay::flush(confidence);
        //vpTime::wait(10); // wait to simulate a local process running on last frame frabbed
      }
    }
    else {
      vpTime::wait(10);
    }
  }while(captureRunning);

  if(displayInit) {
    delete displayX;
    delete displayXConf;
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
