//! \example tutorial-ustk-virtual-server-RF3D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) &&                                                           \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <QApplication>
#include <QtCore/QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_core/usRFToPreScan3DConverter.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF3D.h>
#include <visp3/ustk_core/usMHDSequenceWriter.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);
  QString outputPath;
  if (app.arguments().contains(QString("--output"))) {
    outputPath = app.arguments().at(app.arguments().indexOf(QString("--output")) + 1);
  }

  QThread *grabbingThread = new QThread();

  usNetworkGrabberRF3D *qtGrabber = new usNetworkGrabberRF3D();
  qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 0;      // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // prepare image;
  usVolumeGrabbedInfo<usImageRF3D<short int> > *grabbedVolume;

  // prepare converter
  usImagePreScan3D<unsigned char> preScanImage;
  usRFToPreScan3DConverter converter;

  usMHDSequenceWriter writer;
  if (!outputPath.isEmpty()) {
    writer.setSequenceDirectory(outputPath.toStdString());
  }

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);
  std::cout << "init success" << std::endl;
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedVolume = qtGrabber->acquire();

      std::cout << "MAIN THREAD received volume No : " << grabbedVolume->getVolumeCount() << std::endl;

      // convert RF to pre-scan to save the image
      converter.convert(*grabbedVolume, preScanImage);

      if (!outputPath.isEmpty())
        writer.write(preScanImage, grabbedVolume->getTimeStamps());

    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X  to run this tutorial"
            << std::endl;
  return 0;
}

#endif
