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
      strainImage = elastography->getStrainMap();

      std::cout << "size : " << strainImage.getHeight() << ", " << strainImage.getWidth() << std::endl;

      converter.convert(*grabbedFrame, preScanImage);

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
  std::cout << "You should intall Qt5 (with wigdets and network modules), FFTW and a display graphic system (GDI or "
               "X11) to run this tutorial"
            << std::endl;
  return 0;
}

#endif
