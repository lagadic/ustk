//! \example tutorial-elastography-3D-acquisition.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberRF3D.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  usNetworkGrabberRF3D *qtGrabber = new usNetworkGrabberRF3D();
  qtGrabber->connectToServer();

  std::string preCompressedDirectory, postCompressedDirectory;

  // record directories
  if (qApp->arguments().contains(QString("--preCompressed"))) {
    preCompressedDirectory = qApp->arguments().at(qApp->arguments().indexOf(QString("--preCompressed")) + 1).toStdString();
  }
  else {
    std::cout << "You need to specify the directory to record the pre-compressed volumes with --preCompressed option\n";
    throw vpException(vpException::fatalError, "No output directory specified for pre-compressed volumes !" );
  }
  if (qApp->arguments().contains(QString("--postCompressed"))) {
    postCompressedDirectory = qApp->arguments().at(qApp->arguments().indexOf(QString("--postCompressed")) + 1).toStdString();
  }
  else {
    std::cout << "You need to specify the directory to record the post-compressed volumes with --postCompressed option\n";
    throw vpException(vpException::fatalError, "No output directory specified for post-compressed volumes !" );
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // prepare image;
  usVolumeGrabbedInfo<usImageRF3D<short int> > *grabbedFrame;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);
  qtGrabber->activateRecording(preCompressedDirectory);
  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  qtGrabber->setFramesPerVolume(15);
  qtGrabber->setMotorActivation(true);
  qtGrabber->sendAcquisitionParameters();

  qtGrabber->runAcquisition();

  std::cout << "Start pre-compressed volumes acquisition" << std::endl;

  // our local grabbing loop
  double t0 = vpTime::measureTimeMs();
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;
    }

    else {
      vpTime::wait(10);
    }
  } while (vpTime::measureTimeMs() - t0 < 10000); // we wait 10sec

  qtGrabber->stopAcquisition();
  qtGrabber->disconnectFromServer();

  // HERE we wait 5s that the user moves the probe down to acquire post-compressed volumes
  std::cout << "NOW MOVE THE PROBE DOWN TO ACQUIRE POST-COMPRESSED FRAMES (5sec before next acquisition)" << std::endl;
  vpTime::wait(5000);

  // Then we acquire another set of volumes for post-compression
  delete qtGrabber;
  qtGrabber = new usNetworkGrabberRF3D();
  qtGrabber->connectToServer();

  // setting acquisition parameters
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);
  qtGrabber->activateRecording(postCompressedDirectory);
  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  qtGrabber->setFramesPerVolume(15);
  qtGrabber->setMotorActivation(true);
  qtGrabber->sendAcquisitionParameters();
  qtGrabber->runAcquisition();
  std::cout << "Start post-compressed volumes acquisition" << std::endl;

  // our local grabbing loop
  t0 = vpTime::measureTimeMs();
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;
    }

    else {
      vpTime::wait(10);
      t0 = vpTime::measureTimeMs();
    }
  } while (vpTime::measureTimeMs() - t0 < 10000); // we wait 10sec

  qtGrabber->stopAcquisition();

  return 0;
}

#else
int main()
{
  std::cout << "You should intall FFTW to run this tutorial"
            << std::endl;
  return 0;
}

#endif
