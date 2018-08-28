//! \example tutorial-elastography-2D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) &&     \
    defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QStringList>
#include <QtCore/QThread>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_core/usSequenceWriter.h>
#include <visp3/ustk_elastography/usElastography.h>
#include <visp3/ustk_elastography/usImageElastography.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);
  QString ip;
  // by default we use the virtual server, based on a palpation dataset
  if (app.arguments().contains(QString("--ip")))
    ip = app.arguments().at(qApp->arguments().indexOf(QString("--ip")) + 1);
  else
    ip = QString("127.0.0.1");

  usElastography *elastography = new usElastography;
  elastography->setROI(40, 2700, 50, 500);

  usNetworkGrabberRF2D *qtGrabber = new usNetworkGrabberRF2D();
  qtGrabber->setIPAddress(ip.toStdString());
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
  usImageElastography elastographyImage;
  vpImage<vpRGBa> elastoToDisplay;

  usSequenceWriter<vpImage<vpRGBa> > writer;

// Prepare display
#if defined(VISP_HAVE_X11)
  vpDisplayX *displayElasto = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *displayElasto = NULL;
#endif
  bool displayInit = false;
  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->runAcquisition();

  writer.setSequenceFileName("./elastosequence.xml");
  writer.setImageFileName(std::string("./sequencepreElasto%04d.png"));
  std::cout << "waiting ultrasound initialisation..." << std::endl;

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      elastography->updateRF(*grabbedFrame);
      strainImage = elastography->run();
      std::cout << "strain image size : " << strainImage.getHeight() << ", " << strainImage.getWidth() << std::endl;

      converter.convert(*grabbedFrame, preScanImage);

      elastographyImage.setUltrasoundImage(preScanImage);
      elastographyImage.setStrainMap(strainImage, 270, 40);
      elastoToDisplay = elastographyImage.getElastoImage();
      writer.saveImage(elastoToDisplay);

      // init display
      if (!displayInit && strainImage.getHeight() != 0 && strainImage.getWidth() != 0) {
#if defined(VISP_HAVE_X11)
        displayElasto = new vpDisplayX(elastoToDisplay);
#elif defined(VISP_HAVE_GDI)
        displayElasto = new vpDisplayGDI(elastoToDisplay);
#endif
        displayInit = true;
      }

      // processing display
      if (displayInit) {
        vpDisplay::display(elastoToDisplay);
        vpDisplay::display(strainImage);
        vpDisplay::flush(elastoToDisplay);
        vpDisplay::flush(strainImage);
      }
    }

    else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  if (displayInit) {
    delete displayElasto;
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
