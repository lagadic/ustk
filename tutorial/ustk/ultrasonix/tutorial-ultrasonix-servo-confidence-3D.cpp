//! \example tutorial-ultrasonix-servo-confidence-3D.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/robot/vpRobotViper850.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usScanConverter2D.h>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <visp3/ustk_grabber/usGrabberFrame.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_PTHREAD) && defined(VISP_HAVE_VIPER850)

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
int s_imageType;
vpMutex s_mutex_imageType;
usImagePreScan3D<unsigned char> s_volume_prescan;
int s_frameIndex;
int s_volumeIndex;
vpMutex s_mutex_capture;

vpMutex s_mutex_control_velocity;
vpColVector s_controlVelocity(6, 0.);

//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
vpThread::Return captureFunction(vpThread::Args args)
{
  usGrabberUltrasonix grabber = *((usGrabberUltrasonix *) args);
  usImagePreScan2D<unsigned char> m_frame_prescan;
  usImagePreScan3D<unsigned char> m_prescan_3D;

  //resizing images and saving image type in s_imageType
  if(grabber.getImageType() == us::PRESCAN_3D) {
    s_imageType = us::PRESCAN_3D;
    m_frame_prescan.resize(grabber.getCommunicationsInformations()->m_header.height,
                           grabber.getCommunicationsInformations()->m_header.width);
    m_prescan_3D.resize(grabber.getCommunicationsInformations()->m_header.width,
                        grabber.getCommunicationsInformations()->m_header.height,
                        grabber.getCommunicationsInformations()->m_header.framesPerVolume);
  }
  else
    throw(vpException(vpException::badValue,"use pre-scan 3D images for this example : auto mode on propello"));

  s_mutex_imageType.unlock();

  bool stop_capture_ = false;

  int frameIncrement = -1;
  int frameIndex = 0;
  int volumeIndex = 0;

  //init grabber
  usGrabberFrame<usImagePreScan2D<unsigned char> > grabberFramePreScan;
  grabberFramePreScan.setCommunicationInformation(grabber.getCommunicationsInformations());
  grabberFramePreScan.setTransducerSettings(grabber.getTransducerSettings());

  m_prescan_3D.setMotorSettings(grabber.getMotorSettings());
  m_prescan_3D.setTransducerSettings(grabber.getTransducerSettings());

  m_prescan_3D.setAxialResolution(m_prescan_3D.getDepth()/480);

  while (! stop_capture_) {
    // Capture in progress
    if(grabber.getImageType() == us::PRESCAN_3D) {
      grabberFramePreScan.grabFrame(&m_frame_prescan);

      //update frame increment if we reach last frame in a direction
      if(grabber.getCommunicationsInformations()->m_totFrmIdx % (grabber.getCommunicationsInformations()->m_header.framesPerVolume - 1) == 0)
        frameIncrement = -1 * frameIncrement;

      volumeIndex = grabber.getCommunicationsInformations()->m_totFrmIdx / grabber.getCommunicationsInformations()->m_header.framesPerVolume;
      frameIndex = grabber.getCommunicationsInformations()->m_totFrmIdx % grabber.getCommunicationsInformations()->m_header.framesPerVolume;

      if (volumeIndex % 2)
        frameIndex = grabber.getCommunicationsInformations()->m_header.framesPerVolume - frameIndex - 1;

      //frameNumber += frameIncrement;
      std::cout << "total frame index = " << grabber.getCommunicationsInformations()->m_totFrmIdx << std::endl;
      std::cout << "frame index = " << frameIndex << std::endl;

      //update 3d image
      for(int i=0;i<grabber.getCommunicationsInformations()->m_header.height;i++) {
        for(int j=0;j<grabber.getCommunicationsInformations()->m_header.width;j++) {
          m_prescan_3D(j,i,frameIndex,m_frame_prescan(i,j));
        }
      }
    }

    // Update shared data : 2d frame just grabbed for display, and command
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;

      if(grabber.getImageType() == us::PRESCAN_3D) {
        s_volume_prescan = m_prescan_3D;
        s_frameIndex = frameIndex;
        s_volumeIndex = volumeIndex;
      }
    }
  }

  {
    vpMutex::vpScopedLock lock(s_mutex_capture);
    s_capture_state = capture_stopped;
  }
  std::cout << "End of capture thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded captureFunction]

//! [capture-multi-threaded displayFunction]
vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  int m_imageType;
  usImagePreScan3D<unsigned char> preScan3D_;
  usImagePreScan2D<unsigned char> preScan_;
  usImagePreScan2D<unsigned char> preScanConfidence_;
  usImagePreScan3D<unsigned char> preScanConfidence3D_;
  usScanlineConfidence2D scanlineConfidence;
  int frameIndex = 0;
  int volumeIndex = 0;
  bool frameAlreadyReadInPreviousLoop = false;

  vpColVector imageMoments(2);
  vpColVector s_conf(2);
  double imageSum = 0.0;
  bool initializing = true;
  bool initialized = false;

  double r, theta, phi;

  vpPlot plot(2);
  plot.initGraph(0, 1);
  plot.setTitle(0, "confidence barycenter X");
  plot.setUnitY(0, "error");
  plot.setLegend(0, 0, "time");

  plot.initGraph(1, 1);
  plot.setTitle(1, "confidence barycenter Y");
  plot.setUnitY(1, "error");
  plot.setLegend(1, 0, "time");
  double startTime = vpTime::measureTimeMs();

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_preScan = NULL;
  vpDisplayX *d_conf = NULL;
#endif

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      {
        vpMutex::vpScopedLock lock(s_mutex_imageType);
        m_imageType = s_imageType;
      }
      //capture started
      // Create a copy of the captured frame
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        if(m_imageType == us::PRESCAN_3D) {
          preScan3D_ = s_volume_prescan;
          if((frameIndex == s_frameIndex) && (volumeIndex == s_volumeIndex)) {
            frameAlreadyReadInPreviousLoop = true;
          }
          else {
            frameAlreadyReadInPreviousLoop = false;
            frameIndex = s_frameIndex;
            volumeIndex = s_volumeIndex;
          }
          preScan_.resize(preScan3D_.getBModeSampleNumber(),preScan3D_.getScanLineNumber());
          preScanConfidence3D_.resize(preScan3D_.getDimX(), preScan3D_.getDimY(), preScan3D_.getDimZ());
          //std::cout << "volume loaded in display thread" << std::endl;
        }
        else {
          std::cout << "You must send pre-scan data from ultrasonix station for this example" << std::endl;
          throw(vpException(vpException::badValue));
        }
      }

      //extract new frame (at frameIndex position in 3D image)
      for(unsigned int i=0;i<preScan3D_.getDimX();i++) {
        for(unsigned int j=0;j<preScan3D_.getDimY();j++) {
          preScan_(j,i,preScan3D_(i,j,frameIndex));
        }
      }

      //Confidence map
      scanlineConfidence.run(preScanConfidence_,preScan_);

      double dr = preScan3D_.getTransducerRadius() - preScan3D_.getMotorRadius();

      if (initializing)
      {
        //we do not recompute the command law if the frame hasn't been updated
        if(!frameAlreadyReadInPreviousLoop) {
          std::cout << "[DISPLAY] init imageSum = " << imageSum << ", on frame " << frameIndex << std::endl;
          for (unsigned int j = 0; j < preScan3D_.getScanLineNumber(); ++j) {
            theta = preScan3D_.getScanLinePitch() * j - preScan3D_.getScanLinePitch() / 2.0 * (preScan3D_.getScanLineNumber() - 1.0);
            phi = preScan3D_.getFramePitch() * preScan3D_.getFrameNumber() / 2.0 - preScan3D_.getFramePitch() * preScan3D_.getFrameNumber() / (preScan3D_.getScanLineNumber() * preScan3D_.getFrameNumber() - 1.0) * (j + preScan3D_.getScanLineNumber() * frameIndex);
            for (unsigned int i = 0; i < preScan3D_.getBModeSampleNumber(); ++i)
            {
              r = preScan3D_.getAxialResolution() * i + preScan3D_.getTransducerRadius();
              imageMoments[0] += r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi))
                  * (r * cos(theta) - dr) * theta * preScanConfidence_(i,j);
              imageMoments[1] += r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi))
                  * (r * cos(theta) - dr) * phi * preScanConfidence_(i,j);
              imageSum +=  r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi))
                  * (r * cos(theta) - dr) * preScanConfidence_(i,j);
              preScanConfidence3D_(j,i,frameIndex,preScanConfidence_(i,j));
            }
          }
        }
        std::cout << "frameNumber = " << frameIndex << std::endl;
        if (frameIndex == (preScan3D_.getFrameNumber() - 1))
        {
          std::cout << "End of initialisation (first volume entirely captured)" << std::endl;
          initializing = false;
          initialized = true;
        }
      }
      else if (initialized)
      {
        // unsigned int k = (volumeNumber % 2) ? (preScan3D_.getFrameNumber() - frameNumber - 1) : frameNumber;

        //we do not recompute the command law if the frame hasn't been updated
        if(!frameAlreadyReadInPreviousLoop) {
          std::cout << std::endl << "[DISPLAY] XXXXXXXXXXXXXXXXXX s_frameIndex = " << frameIndex << ", " << std::endl;

          std::cout << "FN = " << preScan3D_.getFrameNumber() << std::endl;

          unsigned int k = (volumeIndex % 2) ? (preScan3D_.getFrameNumber() - frameIndex - 1) : frameIndex;

          std::cout << "k = " << k << std::endl;

          for (unsigned int j = 0; j < preScan3D_.getScanLineNumber(); ++j) {
            theta = preScan3D_.getScanLinePitch() * j - preScan3D_.getScanLinePitch() / 2.0 * (preScan3D_.getScanLineNumber() - 1.0);
            phi = preScan3D_.getFramePitch() * preScan3D_.getFrameNumber() / 2.0 - preScan3D_.getFramePitch() * preScan3D_.getFrameNumber() / (preScan3D_.getScanLineNumber() * preScan3D_.getFrameNumber() - 1.0) * (j + preScan3D_.getScanLineNumber() * k);
            if (volumeIndex % 2) phi = - phi;

            //std::cout << "   theta = " << theta << "     phi = " << phi << std::endl;
            for (unsigned int i = 0; i < preScan3D_.getBModeSampleNumber(); ++i)
            {
              r = preScan3D_.getAxialResolution() * i + preScan3D_.getTransducerRadius();
              imageMoments[0] += r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi)) * (r * cos(theta) - dr) * theta
                  * (preScanConfidence_(i,j) - preScanConfidence3D_(j,i,frameIndex));
              imageMoments[1] += r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi)) * (r * cos(theta) - dr) * phi
                  * (preScanConfidence_(i,j) - preScanConfidence3D_(j,i,frameIndex));
              imageSum += r * vpMath::sqr(cos(theta)) * vpMath::sqr(cos(phi)) * (r * cos(theta) - dr)
                  * (preScanConfidence_(i,j) - preScanConfidence3D_(j,i,frameIndex));
              preScanConfidence3D_(j,i,frameIndex,preScanConfidence_(i,j));
            }
            if(j%32 == 0)
              std::cout << "theta = " << theta << " --- phi = " << phi << std::endl;
          }

          s_conf[0] = imageMoments[0] / imageSum;
          s_conf[1] = - imageMoments[1] / imageSum;

          std::cout << "[DISPLAY] imageSum = " << imageSum << std::endl;
          /*std::cout << "[DISPLAY] s_conf[0]  = " << s_conf[0]  << std::endl;
        std::cout << "[DISPLAY] s_conf[1]  = " << s_conf[1]  << std::endl;*/

          double time = (vpTime::measureTimeMs() - startTime) / 1000.0;
          plot.plot(0,0,time,s_conf[0]);
          plot.plot(1,0,time,s_conf[1]);

          {
            double lambda_c = 5.0;
            vpMutex::vpScopedLock lock(s_mutex_capture);
            s_controlVelocity = 0.0;
            s_controlVelocity[3] = lambda_c * s_conf[0];
            s_controlVelocity[4] = lambda_c * s_conf[1];
          }
        }
      }

      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        if(m_imageType == us::PRESCAN_3D) {
          d_preScan = new vpDisplayX(preScan_);
          d_conf = new vpDisplayX(preScanConfidence_);
          display_initialized_ = true;
        }
#endif
      }
      if(m_imageType == us::PRESCAN_3D) {
        vpDisplay::display(preScan_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(preScan_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(preScan_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(preScan_);

        //Display pre scan confidence
        vpDisplay::display(preScanConfidence_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(preScanConfidence_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(preScanConfidence_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(preScanConfidence_);
      }
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11)
  delete d_preScan;
  delete d_conf;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded displayFunction]


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
  for (int i=0; i< 3; i++)
    lambda[i][i] = 0.02/6;
  // Initialized the torque gain
  for (int i=3; i< 6; i++)
    lambda[i][i] = 1./2;

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
        vpMutex::vpScopedLock lock(s_mutex_capture);
        v_p = s_controlVelocity;
      }

      v_p[0] = 0;
      v_p[1] = 0;
      v_p[2] = 0;
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

      //std::cout << "[CONTROL] v_e = " << v_e.t() << std::endl;

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


//! [capture-multi-threaded mainFunction]
int main(int argc, const char* argv[])
{
  (void) argc;
  (void) argv;
  // Instantiate the grabber
  usGrabberUltrasonix grabber;

  grabber.start();

  // Start the threads
  vpThread thread_capture((vpThread::Fn)captureFunction, (vpThread::Args)&grabber);
  vpThread thread_display((vpThread::Fn)displayFunction);
  vpThread thread_control((vpThread::Fn)controlFunction);

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();
  thread_control.join();

  grabber.stop();

  return 0;
}
//! [capture-multi-threaded mainFunction]

#else
int main()
{
#  ifndef VISP_HAVE_VIPER850
  std::cout << "You need viper 850 robot to run this example" << std::endl;
#  elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
# else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
}

#endif
