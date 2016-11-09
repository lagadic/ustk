//! \example tutorial-sonosite-grabber.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_grabber/usGrabberUltrasonix.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_PTHREAD)

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
usImagePreScan2D<unsigned char> s_frame;
vpMutex s_mutex_capture;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
vpThread::Return captureFunction(vpThread::Args args)
{
  usGrabberUltrasonix<usImagePreScan2D<unsigned char> > grabber = *((usGrabberUltrasonix<usImagePreScan2D<unsigned char> >  *) args);
  usImagePreScan2D<unsigned char> frame_;
  frame_.resize(480,128);
  bool stop_capture_ = false;

  while (! stop_capture_) {
    // Capture in progress
    grabber.grabFrame(&frame_);

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;
      s_frame = frame_;
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
  usImagePreScan2D<unsigned char> preScan_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = NULL;
#endif

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Create a copy of the captured frame
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        preScan_ = s_frame;
      }

      //Settings
      preScan_.setTransducerRadius(0.060);
      preScan_.setTransducerConvexity(true);
      preScan_.setScanLineNumber(128);
      preScan_.setScanLinePitch(vpMath::rad(57/127)); // field of view is 57 deg

      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        d_ = new vpDisplayX(preScan_);
        display_initialized_ = true;
#endif
      }

      // Display the image
      vpDisplay::display(preScan_);

      // Trigger end of acquisition with a mouse click
      vpDisplay::displayText(preScan_, 10, 10, "Click to exit...", vpColor::red);
      if (vpDisplay::getClick(preScan_, false)) {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        s_capture_state = capture_stopped;
      }

      // Update the display
      vpDisplay::flush(preScan_);
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded displayFunction]

//! [capture-multi-threaded mainFunction]
int main(int argc, const char* argv[])
{
  // Instantiate the grabber
  usGrabberUltrasonix<usImagePreScan2D<unsigned char> > grabber;

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
