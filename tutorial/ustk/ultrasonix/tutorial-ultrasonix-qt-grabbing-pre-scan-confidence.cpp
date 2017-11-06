//! \example tutorial-ultrasonix-qt-grabbing-pre-scan-confidence.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))


#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan2D * qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  //prepare image;
  usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >* grabbedFrame;
  usImagePreScan2D<unsigned char> confidence;

  //Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX * display = NULL;
  vpDisplayX * displayConf = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI * display = NULL;
  vpDisplayGDI * displayConf = NULL;
#endif

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
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getWidth() !=0) {
#if defined(VISP_HAVE_X11)
        display = new vpDisplayX(*grabbedFrame);
        displayConf = new vpDisplayX(confidence);
#elif defined(VISP_HAVE_GDI)
        display = new vpDisplayGDI(*grabbedFrame);
        displayConf = new vpDisplayGDI(confidence);
#endif
        qtGrabber->useVpDisplay(display);
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
    delete display;
    delete displayConf;
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
