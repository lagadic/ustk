//! \example tutorial-ultrasonix-qt-grabbing-post-scan.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPostScan2D.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

#include <visp3/io/vpImageIo.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPostScan2D * qtGrabber = new usNetworkGrabberPostScan2D();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  usDataGrabbed<usImagePostScan2D<unsigned char> >* grabbedFrame;

  //Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX * display = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI * display = NULL;
#endif
  bool displayInit = false;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // grab a 640*480 post-scan image
  qtGrabber->setPostScanMode(true);
  qtGrabber->setPostScanHeigh(300);
  qtGrabber->setPostScanWidth(400);

  // set the ultrsound depth to 140 mm
  qtGrabber->setImageDepth(140);

  // set the 4DC7 motor on the middle frame
  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_2);
  qtGrabber->setFramesPerVolume(9);
  qtGrabber->setMotorActivation(true);

  std::cout << "send update" << std::endl;
  qtGrabber->sendAcquisitionParameters();
  std::cout << "end update" << std::endl;
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      std::cout << *grabbedFrame << std::endl;

      char buffer[400];
      sprintf(buffer, "frame%d.png",grabbedFrame->getFrameCount());

      //std::string fileName = "frame" + grabbedFrame->getFrameCount() + ".png";
      vpImageIo::write(*grabbedFrame,buffer);
      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getWidth() !=0) {
#if defined(VISP_HAVE_X11)
        display = new vpDisplayX(*grabbedFrame);
#elif defined(VISP_HAVE_GDI)
        display = new vpDisplayGDI(*grabbedFrame);
#endif
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

  if(displayInit) {
    delete display;
  }

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X  to run this tutorial" << std::endl;
  return 0;
}

#endif
