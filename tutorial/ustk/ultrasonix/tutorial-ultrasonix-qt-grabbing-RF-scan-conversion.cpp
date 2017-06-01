//! \example tutorial-ultrasonix-qt-grabbing-RF-scan-conversion.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(USTK_HAVE_FFTW)

#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberRF * qtGrabber = new usNetworkGrabberRF();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 12; //B-mode = 0, RF = 12

  //prepare image;
  usDataGrabbed<usImageRF2D<short int> >* grabbedFrame;

  //prepare converters
  usImagePreScan2D<unsigned char> preScanImage;
  usImagePostScan2D<unsigned char> postscanImage;
  postscanImage.setHeightResolution(0.0005);
  postscanImage.setWidthResolution(0.0005);

  //ultrasonix 4DC7 probe settings
  postscanImage.setTransducerConvexity(true);
  postscanImage.setTransducerRadius(0.04);
  postscanImage.setScanLineNumber(128);
  postscanImage.setScanLinePitch(0.010625);
  postscanImage.setDepth(0.15);
  preScanImage.setTransducerSettings(postscanImage);

  usRFToPreScan2DConverter converterRF(3840,128);
  usPreScanToPostScan2DConverter scanConverter;

  scanConverter.init(postscanImage,384,128);

  //Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX * display = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI * display = NULL;
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

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      double startTime = vpTime::measureTimeMs();
      //convert RF to pre-scan to display something ...
      converterRF.convert(*grabbedFrame,preScanImage);

      double endRFConvertTime = vpTime::measureTimeMs();
      std::cout << "RF conversion time (sec) = " << (endRFConvertTime - startTime) / 1000.0 << std::endl;

      scanConverter.run(preScanImage,postscanImage);

      double endScanConvertTime = vpTime::measureTimeMs();
      std::cout << "scan-conversion time (sec) = " << (endScanConvertTime - endRFConvertTime) / 1000.0 << std::endl;

      //init display
      if(!displayInit && postscanImage.getHeight() !=0 && postscanImage.getWidth() !=0) {
#if defined(VISP_HAVE_X11)
		  display = new vpDisplayX(postscanImage);
#elif defined(VISP_HAVE_GDI)
		  display = new vpDisplayGDI(postscanImage);
#endif
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(postscanImage);
        vpDisplay::flush(postscanImage);
      }
    }
    else {
      vpTime::wait(10);
    }
  } while(captureRunning);

  if(displayInit) {
    delete display;
  }

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules) to run this tutorial" << std::endl;
  return 0;
}

#endif
