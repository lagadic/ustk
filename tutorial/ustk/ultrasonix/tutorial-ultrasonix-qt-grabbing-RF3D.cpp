//! \example tutorial-ultrasonix-qt-grabbing-RF3D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) &&     \
    defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QtCore/QThread>

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF3D.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberRF3D *qtGrabber = new usNetworkGrabberRF3D();
  qtGrabber->connectToServer();
  // qtGrabber->setVerbose(true);
  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // prepare image;
  usVolumeGrabbedInfo<usImageRF3D<short int> > *grabbedVolume;

  // prepare converter
  usImagePreScan3D<unsigned char> preScanImage;
  usRFToPreScan3DConverter converter;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  qtGrabber->setFramesPerVolume(25);
  qtGrabber->setMotorActivation(true);
  qtGrabber->sendAcquisitionParameters();

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedVolume = qtGrabber->acquire();

      std::cout << "MAIN THREAD received volume No : " << grabbedVolume->getVolumeCount() << std::endl;

      // convert RF to pre-scan to save the image
      converter.convert(*grabbedVolume, preScanImage);

      QString filename = QString("volume") + QString::number(grabbedVolume->getVolumeCount()) + QString(".mhd");
      usImageIo::write(preScanImage, filename.toStdString());
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), FFTW and GDI or X11 to run this tutorial"
            << std::endl;
  return 0;
}

#endif
