//! \example tutorial-ultrasonix-qt-grabbing-Pre-scan.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan3D.h>
#include <visp3/ustk_io/usImageIo.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan3D * qtGrabber = new usNetworkGrabberPreScan3D();
  qtGrabber->setIPAddress("127.0.0.1");
  qtGrabber->setConnection(true);
  //qtGrabber->setVerbose(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 0; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  usVolumeGrabbedInfo<usImagePreScan3D<unsigned char> >* grabbedFrame;


  bool captureRunning = true;
  //qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);
  std::cout << "init success" << std::endl;
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  //our local grabbing loop

  //our grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout <<"MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;

      char buffer[FILENAME_MAX];
      sprintf(buffer, "volumePreScan%d.mhd",grabbedFrame->getVolumeCount());

      usImageIo::write(*grabbedFrame,buffer);
    }
    else {
      vpTime::wait(10);
    }
  } while(captureRunning);

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X  to run this tutorial" << std::endl;
  return 0;
}

#endif
