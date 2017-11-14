//! \example tutorial-ustk-virtual-server-preScan3D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <QtCore/QThread>
#include <QApplication>
#include <QtCore/QStringList>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan3D.h>
#include <visp3/ustk_io/usMHDSequenceWriter.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );
  QString outputPath;
  if(app.arguments().contains(QString("--output"))) {
    outputPath = app.arguments().at(app.arguments().indexOf(QString("--output")) + 1);
  }

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan3D * qtGrabber = new usNetworkGrabberPreScan3D();
  qtGrabber->setIPAddress("127.0.0.1"); //local loop, server must be running on same computer
  qtGrabber->connectToServer();


  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 0; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  usVolumeGrabbedInfo<usImagePreScan3D<unsigned char> >* grabbedFrame;

  usMHDSequenceWriter writer;
  if(!outputPath.isEmpty()) {
    writer.setSequenceDirectory(outputPath.toStdString());
  }

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

  //our grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout <<"MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;

      if(!outputPath.isEmpty())
        writer.write(*grabbedFrame,grabbedFrame->getTimeStamps());

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
