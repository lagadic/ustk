//! \example tutorial-local-grabbing-pre-scan2D-display.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_GRABBER) && defined(VISP_HAVE_VIPER850)

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>
#include <visp3/ustk_gui/usImageDisplayWidget.h>
#include <visp3/ustk_gui/usRobotManualControlWidget.h>
#include <visp3/ustk_gui/usUltrasonixClientWidget.h>
#include <visp3/ustk_gui/usViper850WrapperVelocityControl.h>

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
  usUltrasonixClientWidget *ultrasonixControlWidet = new usUltrasonixClientWidget();

  QWidget *centralWidget = new QWidget();
  QHBoxLayout *mainLayout = new QHBoxLayout();
  mainLayout->addWidget(ultrasonixControlWidet);
  mainLayout->addWidget(widget, 2);
  mainLayout->addWidget(robotControlPanel);
  centralWidget->setLayout(mainLayout);

  // robot control
  QThread *threadRobotControl = new QThread();
  usViper850WrapperVelocityControl viperControl;
  viperControl.moveToThread(threadRobotControl);
  threadRobotControl->start();
  viperControl.run();

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  QMainWindow window;
  window.setCentralWidget(centralWidget);
  window.show();

  QObject::connect(robotControlPanel, SIGNAL(changeVX(int)), &viperControl, SLOT(setXVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVY(int)), &viperControl, SLOT(setYVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVZ(int)), &viperControl, SLOT(setZVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWX(int)), &viperControl, SLOT(setXAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWY(int)), &viperControl, SLOT(setYAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWZ(int)), &viperControl, SLOT(setZAngularVelocity(int)));

  QObject::connect(robotControlPanel, SIGNAL(initClicked()), &viperControl, SLOT(init()));
  QObject::connect(robotControlPanel, SIGNAL(startClicked()), &viperControl, SLOT(run()));
  QObject::connect(robotControlPanel, SIGNAL(stopClicked()), &viperControl, SLOT(stop()));

  QObject::connect(robotControlPanel, SIGNAL(activateAutomaticForceControl()), &viperControl,
                   SLOT(startAutomaticForceControl()));
  QObject::connect(robotControlPanel, SIGNAL(disableAutomaticForceControl()), &viperControl,
                   SLOT(stopAutomaticForceControl()));

  // manage errors
  QObject::connect(&viperControl, SIGNAL(robotError()), robotControlPanel, SLOT(robotErrorSlot()));

  // grabber control
  qRegisterMetaType<QHostAddress>("QHostAddress");
  qRegisterMetaType<usNetworkGrabber::usInitHeaderSent>("usNetworkGrabber::usInitHeaderSent");
  QObject::connect(ultrasonixControlWidet, SIGNAL(connectToServer(QHostAddress)), qtGrabber,
                   SLOT(connectToServer(QHostAddress)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(initAcquisition(usNetworkGrabber::usInitHeaderSent)), qtGrabber,
                   SLOT(initAcquisitionSlot(usNetworkGrabber::usInitHeaderSent)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(runAcquisition()), qtGrabber, SLOT(runAcquisition()));
  QObject::connect(ultrasonixControlWidet, SIGNAL(stopAcquisition()), qtGrabber, SLOT(stopAcquisition()));

  // send new images via qt signal
  qRegisterMetaType<vpImage<unsigned char> >("vpImage<unsigned char>");
  QObject::connect(qtGrabber, SIGNAL(newFrame(vpImage<unsigned char>)), widget,
                   SLOT(updateFrame(vpImage<unsigned char>)));

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
  std::cout << "You should build ustk_gui and ustk_grabber, and have a viper850 robot to run this tutorial"
            << std::endl;
  return 0;
}

#endif
