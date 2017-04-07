//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) & defined(USTK_GRABBER_HAVE_QT5_WIDGETS)

#include <QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>
#include <visp3/ustk_grabber/usNetworkViewerPreScan.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan * qtGrabber = new usNetworkGrabberPreScan();
  qtGrabber->setConnection(true);

  //connect the viewer to the grabber, to update it at each new frame grabbed
  usNetworkViewerPreScan * viewer = new usNetworkViewerPreScan(448,128);
  QObject::connect(qtGrabber,SIGNAL(newFrameArrived(usImagePreScan2D<unsigned char>*)),viewer,SLOT(updateDisplay(usImagePreScan2D<unsigned char>*)));

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.transmitFrequency = 0;
  header.samplingFrequency = 0;
  header.imagingMode = 0; //B-mode = 0
  header.postScanMode = false;
  header.imageDepth = 140; //in mm

  // 2D acquisition
  header.activateMotor = false; //to sweep the motor permanently
  header.motorPosition = 40; // motor in the middle

  /*IF 3D
  header.activateMotor = true;
  header.framesPerVolume = 10;
  header.degreesPerFrame = 3;*/

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules) to run this tutorial" << std::endl;
  return 0;
}

#endif
