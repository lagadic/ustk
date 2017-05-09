//! \example tutorial-ultrasonix-qt-grabbing-RF.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) & defined(USTK_GRABBER_HAVE_QT5_WIDGETS)

#include <QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberRF.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberRF * qtGrabber = new usNetworkGrabberRF();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 12; //B-mode = 0, RF = 12

  //prepare image;
  usDataGrabbed<usImageRF2D<unsigned char> >* grabbedFrame;
  usDataGrabbed<usImageRF2D<unsigned char> > localFrame;

  bool captureRunning = true;

  qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      //local copy for vpDisplay
      localFrame = *grabbedFrame;

      std::cout <<"MAIN THREAD received RF frame No : " << localFrame.getFrameCount() << std::endl;

      //TO DO : convert RF to pre-scan to display something ...


      vpTime::wait(20);
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
  std::cout << "You should intall Qt5 (with wigdets and network modules) to run this tutorial" << std::endl;
  return 0;
}

#endif
