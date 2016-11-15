//! \example tutorial-ultrasonix-confidence-2D.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usScanConverter2D.h>

#include <visp3/ustk_confidence_map/usScanlineConfidence2D.h>

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <visp3/ustk_grabber/usGrabberFrame.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_PTHREAD)

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
int s_imageType;
vpMutex s_mutex_imageType;
usImageRF2D<unsigned char> s_frame_rf;
usImagePreScan2D<unsigned char> s_frame_prescan;
usImagePostScan2D<unsigned char> s_frame_postscan;
vpMutex s_mutex_capture;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
vpThread::Return captureFunction(vpThread::Args args)
{
  usGrabberUltrasonix grabber = *((usGrabberUltrasonix *) args);
  usImageRF2D<unsigned char> m_frame_rf;
  usImagePreScan2D<unsigned char> m_frame_prescan;
  usImagePostScan2D<unsigned char> m_frame_postscan;

  //resizing images and saving image type in s_imageType
  if(grabber.getImageType() == usGrabberUltrasonix::TYPE_RF) {      
    s_imageType = usGrabberUltrasonix::TYPE_RF;
    std::cout << "Error : you must send pre-scan images for this example" << std::endl;
  }
  else if(grabber.getImageType() == usGrabberUltrasonix::TYPE_PRESCAN) {
    s_imageType = usGrabberUltrasonix::TYPE_PRESCAN;
    m_frame_prescan.resize(grabber.getCommunicationsInformations()->m_header.h,
                           grabber.getCommunicationsInformations()->m_header.w);
  }
  else if(grabber.getImageType() == usGrabberUltrasonix::TYPE_POSTSCAN) {
    s_imageType = usGrabberUltrasonix::TYPE_POSTSCAN;
    std::cout << "Error : you must send pre-scan images for this example" << std::endl;
  }
  else
    throw(vpException(vpException::badValue,"unknown type of grabbed image"));

  s_mutex_imageType.unlock();

  bool stop_capture_ = false;

  //init grabber
  usGrabberFrame<usImagePreScan2D<unsigned char> > grabberFramePreScan;
  grabberFramePreScan.setCommunicationInformations(grabber.getCommunicationsInformations());
  grabberFramePreScan.setTransducerSettings(grabber.getTransducerSettings());

  while (! stop_capture_) {
    // Capture in progress
    if(grabber.getImageType() == usGrabberUltrasonix::TYPE_PRESCAN) {
      grabberFramePreScan.grabFrame(&m_frame_prescan);
    }

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;

      if(grabber.getImageType() == usGrabberUltrasonix::TYPE_PRESCAN) {
        s_frame_prescan = m_frame_prescan;
      }
    }
  }

  {
    vpMutex::vpScopedLock lock(s_mutex_capture);
    s_capture_state = capture_stopped;
  }
  return 0;
}
//! [capture-multi-threaded captureFunction]

//! [capture-multi-threaded displayFunction]
vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  int m_imageType;
  usImagePreScan2D<unsigned char> preScan_;
  usImagePostScan2D<unsigned char> postScan_;
  usImagePreScan2D<unsigned char> preScanConfidence_;
  usImagePostScan2D<unsigned char> postScanConfidence_;
  usScanlineConfidence2D scanlineConfidence;
  usScanConverter2D scanConverter;
  bool firstIteration = true;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_preScan = NULL;
  vpDisplayX *d_conf = NULL;
  vpDisplayX *d_postScan = NULL;
  vpDisplayX *d_confPostScan = NULL;
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
       if(m_imageType == usGrabberUltrasonix::TYPE_PRESCAN) {
         preScan_ = s_frame_prescan;
       }
       else {
         std::cout << "You must send pre-scan data from ultrasonix station for this example" << std::endl;
       }
      }


      //Confidence map
      scanlineConfidence.run(preScanConfidence_,preScan_);

      //Scan conversions
      if(firstIteration) {
        scanConverter.init(preScan_,preScan_.getBModeSampleNumber(),preScan_.getScanLineNumber(),0.0005,0.0005);
      }
      scanConverter.run(postScan_,preScan_);
      scanConverter.run(postScanConfidence_,preScanConfidence_);


      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        if(m_imageType == usGrabberUltrasonix::TYPE_PRESCAN) {
          d_preScan = new vpDisplayX(preScan_);
          d_conf = new vpDisplayX(preScanConfidence_);
          d_postScan = new vpDisplayX(postScan_);
          d_confPostScan = new vpDisplayX(postScanConfidence_);
          display_initialized_ = true;
        }
#endif
      }
      if(m_imageType == usGrabberUltrasonix::TYPE_PRESCAN) {
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

        //Display post scan
        vpDisplay::display(postScan_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(postScan_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(postScan_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(postScan_);

        //Display post scan confidence
        vpDisplay::display(postScanConfidence_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(postScanConfidence_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(postScanConfidence_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(postScanConfidence_);
      }
      firstIteration = false;
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11)
  delete d_preScan;
  delete d_conf;
  delete d_confPostScan;
  delete d_postScan;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded displayFunction]

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

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();

  return 0;
}
//! [capture-multi-threaded mainFunction]

#else
int main()
{
#  ifndef VISP_HAVE_V4L2
  std::cout << "You should enable V4L2 to make this example working..." << std::endl;
#  elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#  else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
}

#endif
