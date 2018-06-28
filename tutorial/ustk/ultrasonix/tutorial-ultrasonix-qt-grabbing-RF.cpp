//! \example tutorial-ultrasonix-qt-grabbing-RF.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) &&     \
    defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QThread *grabbingThread = new QThread();

  usNetworkGrabberRF2D *qtGrabber = new usNetworkGrabberRF2D();
  qtGrabber->connectToServer();

  // record option
  if (qApp->arguments().contains(QString("--record"))) {
    qtGrabber->activateRecording(
        qApp->arguments().at(qApp->arguments().indexOf(QString("--record")) + 1).toStdString());
  }

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  if (qApp->arguments().contains(QString("--probeID"))) {
        header.probeId = qApp->arguments().at(qApp->arguments().indexOf(QString("--probeID")) + 1).toInt();
  }
  else
    header.probeId = 15;    // 4DC7 id = 15 by default

  if (qApp->arguments().contains(QString("--slotID"))) {
    header.slotId = qApp->arguments().at(qApp->arguments().indexOf(QString("--slotID")) + 1).toInt();
  }
  else
    header.slotId = 0;      // top slot id = 0 by default

  if (qApp->arguments().contains(QString("--imagingMode"))) {
        header.imagingMode = qApp->arguments().at(qApp->arguments().indexOf(QString("--imagingMode")) + 1).toInt();
  }
  else
    header.imagingMode = 12; // RF = 12

  // prepare image;
  usFrameGrabbedInfo<usImageRF2D<short int> > *grabbedFrame;

  // prepare converter
  usImagePreScan2D<unsigned char> preScanImage;
  usRFToPreScan2DConverter converter;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *display = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *display = NULL;
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

      // convert RF to pre-scan to display something ...
      double t0 = vpTime::measureTimeMs();
      converter.convert(*grabbedFrame, preScanImage);
      double t1 = vpTime::measureTimeMs();
      std::cout << "conversion time = " << t1 - t0 << std::endl;

      // init display
      if (!displayInit && preScanImage.getHeight() != 0 && preScanImage.getWidth() != 0) {
#if defined(VISP_HAVE_X11)
        display = new vpDisplayX(preScanImage);
#elif defined(VISP_HAVE_GDI)
        display = new vpDisplayGDI(preScanImage);
#endif
        displayInit = true;
      }

      // processing display
      if (displayInit) {
        if(vpDisplay::getClick(preScanImage, false))
          captureRunning = false;
        vpDisplay::display(preScanImage);
        vpDisplay::displayText(preScanImage,20,20,std::string("Click to exit..."),vpColor::red);
        vpDisplay::flush(preScanImage);
      }
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  qtGrabber->stopAcquisition();

  if (displayInit) {
    delete display;
  }

  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), FFTW and GDI or X11 to run this tutorial"
            << std::endl;
  return 0;
}

#endif
