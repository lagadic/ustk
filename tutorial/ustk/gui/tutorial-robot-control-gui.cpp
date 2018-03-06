//! \example tutorial-local-grabbing-pre-scan2D-display.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_GRABBER) && defined(VISP_HAVE_VIPER850)

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>
#include <visp3/ustk_gui/usImageDisplayWidget.h>
#include <visp3/ustk_gui/usRobotManualControlWidget.h>
#include <visp3/ustk_gui/usViper850WrapperVelocityContol.h>

#include <QApplication>
#include <QHBoxLayout>
#include <QMainWindow>

int main(int argc, char **argv)
{

  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK display widget"));

  // image
  usImagePreScan2D<unsigned char> *preScan = new usImagePreScan2D<unsigned char>(50, 50);

  // Qt widgets
  usImageDisplayWidget *widget = new usImageDisplayWidget();
  widget->updateFrame(*preScan);

  usRobotManualControlWidget *robotControlPanel = new usRobotManualControlWidget();

  QWidget *centralWidget = new QWidget();
  QHBoxLayout *mainLayout = new QHBoxLayout();
  mainLayout->addWidget(widget, 2);
  mainLayout->addWidget(robotControlPanel);
  centralWidget->setLayout(mainLayout);

  // robot control
  QThread * threadRobotControl = new QThread();
  usViper850WrapperVelocityControl viperControl;
  viperControl.moveToThread(threadRobotControl);
  threadRobotControl->start();
  viperControl.run();


  QMainWindow window;
  window.setCentralWidget(centralWidget);
  window.show();

  QObject::connect(robotControlPanel, SIGNAL(changeVX(int)),&viperControl, SLOT(setXVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVY(int)),&viperControl, SLOT(setYVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVZ(int)),&viperControl, SLOT(setZVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWX(int)),&viperControl, SLOT(setXAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWY(int)),&viperControl, SLOT(setYAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWZ(int)),&viperControl, SLOT(setZAngularVelocity(int)));


  QObject::connect(robotControlPanel, SIGNAL(initClicked()),&viperControl, SLOT(init()));
  QObject::connect(robotControlPanel, SIGNAL(startClicked()),&viperControl, SLOT(run()));
  QObject::connect(robotControlPanel, SIGNAL(stopClicked()),&viperControl, SLOT(stop()));

  // manage errors
  QObject::connect(&viperControl, SIGNAL(robotError()),robotControlPanel, SLOT(robotErrorSlot()));

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  // qtGrabber->setVerbose(true);
  //qtGrabber->setIPAddress("127.0.0.1"); // local loop, server must be running on same computer
  qtGrabber->connectToServer();
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;    // 4DC7 id = 15
  header.slotId = 0;      // top slot id = 0
  header.imagingMode = 0; // B-mode = 0
  qtGrabber->initAcquisition(header);
  qtGrabber->runAcquisition();
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  //send new images via qt signal
  qRegisterMetaType<vpImage<unsigned char> >("vpImage<unsigned char>");
  QObject::connect(qtGrabber, SIGNAL(newFrame(vpImage<unsigned char> )),widget, SLOT(updateFrame(vpImage<unsigned char> )));

  app.exec();

  grabbingThread->exit();

  delete preScan;
  delete widget;
  delete robotControlPanel;
  delete centralWidget;
  delete mainLayout;
  delete grabbingThread;
  delete qtGrabber;

  return 0;
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_grabber, and have a viper850 robot to run this tutorial" << std::endl;
  return 0;
}

#endif
