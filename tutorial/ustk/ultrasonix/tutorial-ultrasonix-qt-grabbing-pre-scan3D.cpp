//! \example tutorial-ultrasonix-qt-grabbing-pre-scan3D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan3D.h>

#include <visp3/ustk_core/usImageIo.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  usNetworkGrabberPreScan3D *qtGrabber = new usNetworkGrabberPreScan3D();
  qtGrabber->connectToServer();

  // record option
  if (qApp->arguments().contains(QString("--record"))) {
    qtGrabber->activateRecording(
        qApp->arguments().at(qApp->arguments().indexOf(QString("--record")) + 1).toStdString());
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;    // 4DC7 id = 15
  header.slotId = 0;      // top slot id = 0
  header.imagingMode = 0; // B-mode = 0

  // prepare image;
  usVolumeGrabbedInfo<usImagePreScan3D<unsigned char> > *grabbedFrame;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  qtGrabber->setFramesPerVolume(25);
  qtGrabber->setMotorActivation(true);
  qtGrabber->sendAcquisitionParameters();

  // Send the command to run the acquisition
  qtGrabber->runAcquisition();

  // our grabbing loop
  do {
    grabbedFrame = qtGrabber->acquire();

    std::cout << "MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;

    char buffer[FILENAME_MAX];
    snprintf(buffer, FILENAME_MAX, "volumePreScan%d.mhd", grabbedFrame->getVolumeCount());

    usImageIo::write(*grabbedFrame, buffer);

  } while (captureRunning);

  qtGrabber->stopAcquisition();

  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial"
    << std::endl;
  return 0;
}

#endif
