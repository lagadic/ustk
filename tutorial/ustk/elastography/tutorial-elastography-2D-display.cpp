//! \example tutorial-elastography2D.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) &&     \
    defined(USTK_HAVE_FFTW)

#include <QApplication>
#include <QtCore/QThread>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QToolBar>

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_elastography/UsQtDisplay.h>
#include <visp3/ustk_elastography/usElastography.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF2D.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  QWidget *window = new QWidget;

  window->setWindowTitle("Demo with visp_ustk");
  QHBoxLayout *L0 = new QHBoxLayout;
  QToolBar *B0 = new QToolBar;
  QIcon ico_net(QPixmap(":/icons/icons/transfer_on.png"));
  ico_net.addPixmap(QPixmap(":/icons/icons/transfer_off.png"), QIcon::Normal, QIcon::On);
  QAction *a_openNet = new QAction(ico_net, "open Network", window);
  a_openNet->setCheckable(true);
  // Shared pointers
  QSharedPointer<usImageRF2D<short int> > s_RFIm = QSharedPointer<usImageRF2D<short int> >::create();
  QSharedPointer<vpMatrix> s_StrainIm = QSharedPointer<vpMatrix>::create();
  QSharedPointer<usImagePreScan2D<unsigned char> > s_PreScanIm =
      QSharedPointer<usImagePreScan2D<unsigned char> >::create();

  // Objects from library
  UsQtDisplay *usDisp = new UsQtDisplay;

  usElastography *elastography = new usElastography;

  elastography->setCommonSharedRFImage(s_RFIm);
  elastography->setCommonSharedStrainImage(s_StrainIm);

  // ######### Virtual server activated ######## //
  // Adding the shared pointers to needed objects
  usDisp->setCommonSharedStrainImage(s_StrainIm);
  usDisp->setCommonSharedPreScanImage(s_PreScanIm);
  usDisp->setCommonSharedRFImage(s_RFIm);

  // Fisrt lateral toolbar
  B0->setOrientation(Qt::Vertical);
  B0->setMinimumSize(24, 410);
  B0->addAction(a_openNet);
  B0->addSeparator();

  // Second column
  L0->addWidget(B0);
  L0->addWidget(usDisp);

  window->setLayout(L0);

  // End acq
  QObject::connect(elastography, SIGNAL(StrainMapComp()), usDisp, SLOT(DrawElasto()));
  QObject::connect(usDisp, SIGNAL(sPattern(int, int, int, int)), elastography, SLOT(setROI(int, int, int, int)));

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

  // prepare converter
  usRFToPreScan2DConverter converter;
  usImagePreScan2D<unsigned char> preScanImage;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  window->show();

  // our local grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      *(s_RFIm.data()) = *grabbedFrame;
      // elastography->setRF(*grabbedFrame);

      converter.convert(*grabbedFrame, preScanImage);

      *(s_PreScanIm.data()) = preScanImage;
      usDisp->chgImage();
      elastography->setRF();

    }

    else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
