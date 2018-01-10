//! \example tutorial-elastography2D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) &&     \
    defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QtCore/QThread>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_elastography/usElastography.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  usElastography *elastography = new usElastography;
  elastography->setROI(40, 2500, 50, 500);
  elastography->start();

  QThread *grabbingThread = new QThread();

  usNetworkGrabberRF2D *qtGrabber = new usNetworkGrabberRF2D();
  qtGrabber->setIPAddress("127.0.0.1");
  qtGrabber->connectToServer();

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0, RF = 12

  // prepare image;
  usFrameGrabbedInfo<usImageRF2D<short int> > *grabbedFrame;
  usImagePreScan2D<unsigned char> preScanImage;

  usRFToPreScan2DConverter converter;

  // prepare converter
  vpImage<unsigned char> strainImage;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *displayEcho = NULL;
  vpDisplayX *displayElas = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *displayEcho = NULL;
  vpDisplayGDI *displayElas = NULL;
#endif
  bool displayInit = false;
  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // update motor position
  // qtGrabber->setMotorPosition(37);
  // qtGrabber->sendAcquisitionParameters();

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      elastography->setRF(*grabbedFrame);
      vpMatrix strainMap = elastography->getStrainMap();
      // std::cout << "strain rows: " << strainMap.getRows() << ", strain cols: " << strainMap.getCols() << std::endl;
      strainImage.resize(strainMap.getRows(), strainMap.getCols());
      std::cout << "strain img height: " << strainImage.getHeight() << ", strain width: " << strainImage.getWidth()
                << std::endl;

      for (unsigned int i = 0; i < strainImage.getHeight(); i++) {
        for (unsigned int j = 0; j < strainImage.getWidth(); j++) {
          strainImage[i][j] = (unsigned char)(strainMap[i][j] * 254);
          // if (strainImage[i][j] < 30)
          // std::cout << "black at : " << i << ", " << j << std::endl;
        }
      }

      converter.convert(*grabbedFrame, preScanImage);
      std::cout << "RF dims : " << grabbedFrame->getHeight() << ", " << grabbedFrame->getWidth() << std::endl;
      std::cout << "PS dims : " << preScanImage.getHeight() << ", " << preScanImage.getWidth() << std::endl;

      // init display
      if (!displayInit && strainImage.getHeight() != 0 && strainImage.getWidth() != 0) {
#if defined(VISP_HAVE_X11)
        displayEcho = new vpDisplayX(preScanImage);
        displayElas = new vpDisplayX(strainImage);
#elif defined(VISP_HAVE_GDI)
        displayEcho = new vpDisplayGDI(preScanImage);
        displayElas = new vpDisplayGDI(strainImage);
#endif
        displayInit = true;
      }

      // processing display
      if (displayInit) {

        vpDisplay::display(preScanImage);
        vpDisplay::displayRectangle(preScanImage, 250, 40, 50, 50, vpColor::red);
        vpDisplay::flush(preScanImage);
        vpDisplay::display(strainImage);
        vpDisplay::flush(strainImage);
      }
    }

    else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  if (displayInit) {
    delete displayElas;
    delete displayEcho;
  }
  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
