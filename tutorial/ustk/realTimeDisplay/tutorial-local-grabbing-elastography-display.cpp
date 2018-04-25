//! \example tutorial-local-grabbing-elastography-display.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_GRABBER)

#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>
#include <visp3/ustk_gui/usElastographyDisplayWidget.h>
#include <visp3/ustk_gui/usImageElastographyCreationWrapper.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{

  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK display widget"));

  // image
  // usFrameGrabbedInfo<usImageRF2D<short int> > *grabbedFrame;
  usImagePreScan2D<unsigned char> preScanImage;
  vpImage<vpRGBa> elastoImage(50, 50, vpRGBa(0, 200, 0, 254));
  vpImage<unsigned char> strainImage(50, 50);

  // Qt widgets
  usElastographyDisplayWidget *widget = new usElastographyDisplayWidget();
  widget->updateFrame(elastoImage);
  QMainWindow window;
  window.setCentralWidget(widget);
  window.show();

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberRF2D *qtGrabber = new usNetworkGrabberRF2D();
  // qtGrabber->setVerbose(true);
  qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;     // 4DC7 id = 15
  header.slotId = 0;       // top slot id = 0
  header.imagingMode = 12; // B-mode = 0
  qtGrabber->initAcquisition(header);
  qtGrabber->runAcquisition();
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  // converter & elasto processes
  /*usElastography elastoProcess;
  elastoProcess.setROI(40, 3200, 50, 500);
  usImageElastography elastographyImageCreation;
  usRFToPreScan2DConverter converter;*/
  QThread *elastoThread = new QThread();
  usImageElastographyCreationWrapper *elastoGenerator = new usImageElastographyCreationWrapper;
  elastoGenerator->setROI(270, 40, 500, 50);
  elastoGenerator->moveToThread(elastoThread);
  elastoThread->start();

  qRegisterMetaType<vpImage<vpRGBa> >("vpImage<vpRGBa>");
  QObject::connect(elastoGenerator, SIGNAL(elastographyImageReady(vpImage<vpRGBa>)), widget,
                   SLOT(updateFrame(vpImage<vpRGBa>)));

  qRegisterMetaType<usImageRF2D<short int> >("usImageRF2D<short int>&");
  QObject::connect(qtGrabber, SIGNAL(newFrame(usImageRF2D<short int> &)), elastoGenerator,
                   SLOT(updateFrame(usImageRF2D<short int> &)));

  usImageRF2D<short int> rfFrame;
  // our grabbing loop
  /*do {
    if (qtGrabber->isFirstFrameAvailable()) {
      //rfFrame = (*qtGrabber->acquire());
      //std::cout << "new Frame acquired : " << rfFrame << std::endl;

      //->updateFrame(rfFrame);

      //std::cout << "end loop" << std::endl;
      // update display
      // widget->updateFrame(elastoImage);
    }
  } while (window.isVisible());*/

  app.exec();
  // disconnect from server
  qtGrabber->disconnectFromServer();
  grabbingThread->quit();
  return 0;
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_grabber to run this tutorial" << std::endl;
  return 0;
}

#endif
