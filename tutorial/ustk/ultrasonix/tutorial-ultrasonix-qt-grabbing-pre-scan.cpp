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

  usNetworkGrabberPreScan * qtGrabber = new usNetworkGrabberPreScan();

  //QThread * thread = new QThread();
  qtGrabber->setConnection(true);
  //qtGrabber->moveToThread(thread);

  //connect the viewer to the grabber, to update it at each new frame grabbed
  usNetworkViewerPreScan * viewer = new usNetworkViewerPreScan(128,448);
  viewer->setGrabber(qtGrabber);

  // sending acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.transmitFrequency = 0;
  header.samplingFrequency = 0;
  header.imagingMode = 0; //B-mode = 0
  header.postScanMode = false;
  header.imageDepth = 140; //in mm

  header.activateMotor = false; //to sweep the motor permanently
  header.motorPosition = 40; // motor in the middle

  /*IF 3D
  header.framesPerVolume = 10;
  header.degreesPerFrame = 3;*/

  qtGrabber->initAcquisition(header);

  app.exec();

  std::cout << "ending application" << std::endl;
  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules) to run this tutorial" << std::endl;
  return 0;
}

#endif
