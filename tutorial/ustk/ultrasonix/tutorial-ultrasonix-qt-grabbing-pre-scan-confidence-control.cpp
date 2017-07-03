//! \example tutorial-ultrasonix-qt-grabbing-pre-scan-confidence-control.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_VIPER850)

#include <QtCore/QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotViper850.h>

// Shared vars

vpMutex s_mutex_control_velocity;
vpColVector s_controlVelocity(6, 0.);

vpMutex s_mutex_capture;
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;


//! [capture-multi-threaded robot control function]
vpThread::Return controlFunction(vpThread::Args args)
{
  (void) args;
  vpRobotViper850 robot ;

  vpMatrix eJe; // robot jacobian

  // Transformation from end-effector frame to the force/torque sensor
  // Note that the end-effector frame is located on the lower part of
  // male component of the tool changer.
  vpHomogeneousMatrix eMs;
  eMs[2][3] = -0.062; // tz = -6.2cm

  // Transformation from force/torque sensor to the probe frame from where
  // we want to control the robot
  vpHomogeneousMatrix sMp;

  // Transformation from force/torque sensor to the end-effector frame
  vpHomogeneousMatrix sMe;
  eMs.inverse(sMe);

  // Build the transformation that allows to convert a velocity in the
  // end-effector frame to the FT sensor frame
  vpVelocityTwistMatrix sVe;
  sVe.buildFrom(sMe);

  vpColVector sHs(6); // force/torque sensor measures
  vpColVector sHs_star(6); // force/torque sensor desired values in sensor frame
  vpColVector pHp_star(6); // force/torque sensor desired values in probe frame
  vpColVector gHg(6); // force/torque due to the gravity
  vpMatrix lambda(6,6);
  // Position of the cog of the object attached after the sensor in the sensor frame
  vpTranslationVector stg;
  vpColVector sHs_bias(6); // force/torque sensor measures for bias

  // Cartesian velocities corresponding to the force/torque control in the
  // sensor frame
  vpColVector v_s(6);
  // Joint velocities corresponding to the force/torque control
  vpColVector q_dot(6);

  // Initialized the force gain
  lambda = 0;
  for (int i=0; i< 3; i++)
    lambda[i][i] = 0.05/6;
  // Initialized the torque gain
  for (int i=3; i< 6; i++)
    lambda[i][i] = 1.2/2;

  // Initialize the desired force/torque values
  pHp_star = 0;
  pHp_star[2] = 3; // Fz = 3N
  //
  // Case of the C65 US probe
  //
  // Set the probe frame control
  sMp[2][3] = 0.262;  // tz = 26.2cm

  // Init the force/torque due to the gravity
  gHg[2] = -(0.696+0.476)*9.81; // m*g

  // Position of the cog of the object attached after the sensor in the sensor frame
  stg[0] = 0;
  stg[1] = 0;
  stg[2] = 0.088; // tz = 88.4mm


  vpRotationMatrix sRp;
  sMp.extract(sRp);
  vpTranslationVector stp;
  sMp.extract(stp);

  vpHomogeneousMatrix eMp = eMs * sMp;
  vpVelocityTwistMatrix eVp(eMp);

  // Get the position of the end-effector in the reference frame
  vpColVector q;
  vpHomogeneousMatrix fMe;
  vpHomogeneousMatrix fMs;
  vpRotationMatrix sRf;
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
  robot.get_fMe(q, fMe);
  // Compute the position of the sensor frame in the reference frame
  fMs = fMe * eMs;
  vpHomogeneousMatrix sMf;
  fMs.inverse(sMf);
  sMf.extract(sRf);

  // Build the transformation that allows to convert the forces due to the
  // gravity in the sensor frame
  vpForceTwistMatrix sFg(sMf); // Only the rotation part is to consider
  // Modify the translational part
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      sFg[i+3][j] = (stg.skew()*sRf)[i][j];


  // Build the transformation that allows to convert a FT expressed in the
  // FT probe frame into the sensor frame
  vpForceTwistMatrix sFp(sMp);

  // Bias the force/torque sensor
  std::cout << "\nBias the force/torque sensor...\n " << std::endl;
  robot.biasForceTorqueSensor() ;

  // Set the robot to velocity control
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter = 0;
  t_CaptureState capture_state_;

  std::cout << "Starting control loop..." << std::endl;
  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {

      // Get the force/torque measures from the sensor
      sHs = robot.getForceTorque() ;

      // Multiply the measures by -1 to get the force/torque exerced by the
      // robot to the environment.
      sHs *= -1;

      // Update the gravity transformation matrix
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
      robot.get_fMe(q, fMe);
      // Compute the position of the sensor frame in the reference frame
      fMs = fMe * eMs;
      // Compute the inverse transformation
      fMs.inverse(sMf);
      sMf.extract(sRf);
      // Update the transformation that allows to convert the forces due to the
      // gravity in the sensor frame
      sFg.buildFrom(sMf); // Only the rotation part is to consider
      // Modify the translational part
      for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
          sFg[i+3][j] = (stg.skew()*sRf)[i][j];

      if (iter == 0) {
        sHs_bias = sHs - sFg * gHg;
      }

      // Compute rotation in probe frame from control velocity deduced of the confidence map barycenter
      vpColVector v_p;
      {
        vpMutex::vpScopedLock lock(s_mutex_control_velocity);
        v_p = s_controlVelocity;
      }

      v_p[0] = 0;
      v_p[1] = 0;
      v_p[4] = 0;
      v_p[5] = 0;

      // Compute the force/torque control law in the sensor frame
      v_s = lambda*(sFp * pHp_star - (sHs - sFg * gHg - sHs_bias) );

      v_s[0] = 0.0;
      v_s[1] = 0.0;
      v_s[3] = 0.0;
      v_s[4] = 0.0;
      v_s[5] = 0.0;

      vpVelocityTwistMatrix eVs;
      sVe.inverse(eVs);

      vpColVector v_e = eVs * v_s + eVp * v_p;

      // Get the robot jacobian eJe
      robot.get_eJe(eJe);

      // Compute the joint velocities to achieve the force/torque control
      q_dot = eJe.pseudoInverse() * v_e;

      // Send the joint velocities to the robot
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, q_dot) ;

      iter ++;
    }
    vpTime::wait(1); // 5
  } while(capture_state_ != capture_stopped);

  std::cout << "End of control thread" << std::endl;
  return 0;
}


int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan2D * qtGrabber = new usNetworkGrabberPreScan2D();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.imagingMode = 0; //B-mode = 0

  //prepare image;
  usDataGrabbed<usImagePreScan2D<unsigned char> >* grabbedFrame;
  usDataGrabbed<usImagePreScan2D<unsigned char> > localFrame;
  usImagePreScan2D<unsigned char> confidence;

  //Prepare display
  vpDisplay * display = NULL;
  vpDisplay * displayConf = NULL;
  bool displayInit = false;

  //prepare confidence
  usScanlineConfidence2D confidenceProcessor;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->setMotorPosition(32); //middle
  qtGrabber->sendAcquisitionParameters();

  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  //start robot control thread
  vpThread thread_control((vpThread::Fn)controlFunction);

  std::cout << "waiting ultrasound initialisation..." << std::endl;

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();
      confidenceProcessor.run(confidence, *grabbedFrame);

      // Confidence barycenter computation

      //sum first
      double I_sum = 0.0;
      for (unsigned int i = 0; i < confidence.getHeight(); ++i)
        for (unsigned int j = 0; j < confidence.getWidth(); ++j)
          I_sum += static_cast<double>(confidence[i][j]);

      double yc = 0.0;
      for (unsigned int x = 0; x < confidence.getHeight(); ++x)
        for (unsigned int y = 0; y < confidence.getWidth(); ++y) {
          yc += y * confidence[x][y];
        }
      yc /= I_sum;

      double tc = yc * confidence.getScanLinePitch() - confidence.getFieldOfView() / 2.0;

      {
        vpMutex::vpScopedLock lock(s_mutex_control_velocity);
        s_controlVelocity = 0.0;
        s_controlVelocity[3] = 0.5 * tc;
      }

      s_mutex_capture.lock();
      s_capture_state = capture_started;
      s_mutex_capture.unlock();

      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;

      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getWidth() !=0) {
#ifdef VISP_HAVE_X11
        display = new vpDisplayX(*grabbedFrame);
        displayConf = new vpDisplayX(confidence, (*grabbedFrame).getWidth()+60, 10);
#elif defined(VISP_HAVE_GDI)
        display = new vpDisplayGDI(*grabbedFrame);
        displayConf = new vpDisplayGDI(confidence, (*grabbedFrame).getWidth()+60, 10);
#endif
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(*grabbedFrame);
        vpDisplay::display(confidence);

        //display target barycenter (image center)
        vpDisplay::displayText(confidence, 10, 10, "target", vpColor::green);
        vpDisplay::displayLine(confidence,
                               0, confidence.getWidth()/2-1,
                               confidence.getHeight()-1, confidence.getWidth()/2-1,
                               vpColor::green);

        //dispay current confidence barycenter
        vpDisplay::displayText(confidence, 25, 10, "current", vpColor::red);
        vpDisplay::displayLine(confidence,
                               0, static_cast<int>(yc),
                               confidence.getHeight()-1, static_cast<int>(yc),
                               vpColor::red);
        if(vpDisplay::getClick(confidence, false))
          captureRunning = false;
        if(vpDisplay::getClick(*grabbedFrame, false))
          captureRunning = false;

        vpDisplay::flush(*grabbedFrame);
        vpDisplay::flush(confidence);
        vpTime::wait(10); // wait to simulate a local process running on last frame frabbed
      }
    }
    else {
      vpTime::wait(100);
    }
  } while(captureRunning);

  std::cout << "stop capture" << std::endl;
  if(displayInit) {
    if (display)
      delete display;

    if (displayConf)
      delete displayConf;
  }

  // move up the robot arm
  {
    vpMutex::vpScopedLock lock(s_mutex_control_velocity);
    s_controlVelocity = 0.0;
    s_controlVelocity[2] = -0.05; //move up in probe frame
  }
  vpTime::wait(500);
  {
    vpMutex::vpScopedLock lock(s_mutex_control_velocity);
    s_controlVelocity = 0.0;
  }

  s_mutex_capture.lock();
  s_capture_state = capture_stopped;
  s_mutex_capture.unlock();
  thread_control.join();

  // end grabber
  qtGrabber->disconnect();
  grabbingThread->exit();
  app.quit();

  std::cout << "end main thread" << std::endl;


  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial" << std::endl;
  return 0;
}

#endif
