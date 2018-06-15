//! \example tutorial-image-display-qtquickoverlayServoing.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING) &&                          \
    defined(VISP_HAVE_VIPER850)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>
#include <visp3/ustk_gui/usImageDisplayWidgetQmlOverlayServoing.h>
#include <visp3/ustk_gui/usRectangleVisualServoingController.h>
#include <visp3/ustk_gui/usRobotManualControlWidget.h>
#include <visp3/ustk_gui/usTracker2DQtWrapper.h>
#include <visp3/ustk_gui/usUltrasonixClientWidget.h>
#include <visp3/ustk_gui/usViper850WrapperVelocityControl.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK tracking 2D demo"));

  usImageDisplayWidgetQmlOverlayServoing *widget = new usImageDisplayWidgetQmlOverlayServoing();

  usUltrasonixClientWidget *ultrasonixControlWidet = new usUltrasonixClientWidget();
  usRobotManualControlWidget *robotControlPanel = new usRobotManualControlWidget();

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

  // 2D ROI tracking & confidence based controller
  usRectangleVisualServoingController *visualServoingController = new usRectangleVisualServoingController();
  QThread *servoingThread = new QThread();
  visualServoingController->moveToThread(servoingThread);
  servoingThread->start();

  // grabber
  QThread *grabbingThread = new QThread();
  usNetworkGrabberPreScan2D *qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->activateRecording("/home/usData/trackingRoiSequence/");
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  // usTracker2DQtWrapper *tracker = new usTracker2DQtWrapper();

  // manual robot controls
  QObject::connect(robotControlPanel, SIGNAL(changeVX(int)), &viperControl, SLOT(setXVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVY(int)), &viperControl, SLOT(setYVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeVZ(int)), &viperControl, SLOT(setZVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWX(int)), &viperControl, SLOT(setXAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWY(int)), &viperControl, SLOT(setYAngularVelocity(int)));
  QObject::connect(robotControlPanel, SIGNAL(changeWZ(int)), &viperControl, SLOT(setZAngularVelocity(int)));

  QObject::connect(robotControlPanel, SIGNAL(initRobot()), &viperControl, SLOT(init()));
  QObject::connect(robotControlPanel, SIGNAL(startRobot()), &viperControl, SLOT(run()));
  QObject::connect(robotControlPanel, SIGNAL(stopRobot()), &viperControl, SLOT(stop()));

  QObject::connect(robotControlPanel, SIGNAL(activateAutomaticForceControl()), &viperControl,
                   SLOT(startAutomaticForceControl()));
  QObject::connect(robotControlPanel, SIGNAL(disableAutomaticForceControl()), &viperControl,
                   SLOT(stopAutomaticForceControl()));

  // manage errors
  QObject::connect(&viperControl, SIGNAL(robotError()), robotControlPanel, SLOT(robotErrorSlot()));

  // controls to start / stop tracking & servoing
  qRegisterMetaType<vpRectOriented>("vpRectOriented");
  QObject::connect(widget, SIGNAL(startTrackingRect(vpRectOriented)), visualServoingController,
                   SLOT(initTracker(vpRectOriented)));
  QObject::connect(widget, SIGNAL(stopTrackingRect()), visualServoingController, SLOT(stopTracking()));
  QObject::connect(widget, SIGNAL(startServoingRect()), visualServoingController, SLOT(activateController()));
  QObject::connect(widget, SIGNAL(stopServoingRect()), visualServoingController, SLOT(disactivateController()));

  // grabber control
  qRegisterMetaType<QHostAddress>("QHostAddress");
  qRegisterMetaType<usNetworkGrabber::usInitHeaderSent>("usNetworkGrabber::usInitHeaderSent");
  QObject::connect(ultrasonixControlWidet, SIGNAL(connectToServer(QHostAddress)), qtGrabber,
                   SLOT(connectToServer(QHostAddress)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(initAcquisition(usNetworkGrabber::usInitHeaderSent)), qtGrabber,
                   SLOT(initAcquisitionSlot(usNetworkGrabber::usInitHeaderSent)));
  QObject::connect(ultrasonixControlWidet, SIGNAL(center3DProbeMotor()), qtGrabber, SLOT(center3DProbeMotor()));
  QObject::connect(ultrasonixControlWidet, SIGNAL(runAcquisition()), qtGrabber, SLOT(runAcquisition()));
  QObject::connect(ultrasonixControlWidet, SIGNAL(stopAcquisition()), qtGrabber, SLOT(stopAcquisition()));

  // send new images via qt signal
  qRegisterMetaType<usImagePostScan2D<unsigned char> >("usImagePreScan2D<unsigned char>");
  qRegisterMetaType<usImagePostScan2D<unsigned char> >("usImagePostScan2D<unsigned char>");
  QObject::connect(qtGrabber, SIGNAL(newFrame(usImagePreScan2D<unsigned char>)), visualServoingController,
                   SLOT(updateImage(usImagePreScan2D<unsigned char>)));
  QObject::connect(visualServoingController, SIGNAL(newPostScanFrame(usImagePostScan2D<unsigned char>)), widget,
                   SLOT(updateFrame(usImagePostScan2D<unsigned char>)));

  // updates the GUI based on the tracking output
  QObject::connect(visualServoingController, SIGNAL(newRectTracked(vpRectOriented)), widget,
                   SLOT(updateRectPosition(vpRectOriented)));

  // send robot velocities from visualServoingController
  QObject::connect(visualServoingController, SIGNAL(updatePobeXVelocity(int)), &viperControl, SLOT(setXVelocity(int)));
  QObject::connect(visualServoingController, SIGNAL(updateProbeZOrientation(int)), &viperControl,
                   SLOT(setZAngularVelocity(int)));

  QMainWindow window;
  window.setCentralWidget(centralWidget);
  window.resize(1200,650);
  window.show();

  return app.exec();
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_template_tracking to run this tutorial" << std::endl;
  return 0;
}

#endif
