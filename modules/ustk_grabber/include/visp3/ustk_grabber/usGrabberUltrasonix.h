/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Alexandre Krupa
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usGrabberUltrasonix.h
 * @brief Ultrasonix data grabber.
 * @author Pierre Chatelain
 */

#ifndef __usGrabberUltrasonix_h_
#define __usGrabberUltrasonix_h_

#ifndef _WIN32

#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <unistd.h> // for close(int socket_id)
#include <cerrno>

#include <visp3/core/vpTime.h>

#include <visp3/ustk_core/us.h>
//usImages
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

#define TCP
#define PORT 5555
#define NUM_FRAME_OFFSET 2


// 4DC7-3 probe properties
#define US_4DC7_DEG_PER_LINE 0.608767657441f
//#define US_4DC7_PROBE_RADIUS 0.04f // m
//#define US_4DC7_MOTOR_RADIUS 0.02725f // m
#define US_4DC7_BSAMPLE_DISTANCE 0.000308f // m


/**
 * @class usGrabberUltrasonix
 * @brief Ultrasonix data grabber.
 * @ingroup module_ustk_grabber
 *
 * Class for Ultrasonix data grabber.
 */
class VISP_EXPORT usGrabberUltrasonix {
 public:

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct SocketHeader3D
  {
    double startTime;
    int imageType; // 0: prescan 1: postscan 2: rf
    int volumeNumber; // Number of volumes in the sequence
    int framesPerVolume; // Frames per volumes
    int width; // Frame width (scan line number)
    int height; // Frame height (sample number)
    int sampleSize; // Sample size (in bits)
    int degPerFr; // TO RENAME ! Frame pitch (check if the value is correct) (x 1000?)
    int BSampleFreq; // Sampling frequency (Hz)
    int ProbeElementPitch; //micron
    int ProbeRadius; //micron
    int MotorRadius; //micron
    int framerate; // Frames per second
  };
#endif //DOXYGEN_SHOULD_SKIP_THIS

  struct usGrabberCommunicationInformations
  {
  int m_serv_fd;
  int m_cli_fd;
  socklen_t m_clilen;
  struct sockaddr_in m_serv_addr;
  struct sockaddr_in m_cli_addr;
  SocketHeader3D m_header;
  double m_localTime;
  double m_deviceTime;
  int m_szFrm;
  int m_szVol;
  unsigned char *m_voldata;
  int m_byteSizeFrmInf;
  char *m_frmInf;
  int m_totFrmIdx;
  double m_frmTime;
  int m_iter;
  int m_previousFrmIdx;
  int m_frmIdx;
  double m_frameRate;
  };

  /**
   * Constructor
   */
  usGrabberUltrasonix();

  /**
   * Destructor.
   */
   ~usGrabberUltrasonix();

  usGrabberCommunicationInformations* getCommunicationsInformations(){return &m_communicationInormations;}

  us::ImageType getImageType() const {return m_imageType;}

  /**
   * Get the frame rate.
   */
  double getFrameRate() const;

  /**
   * Get the socket identifier.
   */
  int getSocket() const;

  usTransducerSettings getTransducerSettings() const {return m_transducerSettings;}

  /**
   * Initialization method.
   *
   * The method performs the following tasks:
   *  - Connection to the Ultrasonix scanner via TCP.
   *    The methods blocks until a connection is established.
   *  - If the connection is successful, initialization of the usData container with the corresponding type.
   *    This will consume the first frame sent by the scanner to determine the data type.
   *    Currently supported types are 2D and 3D pre-scan, and 2D post-scan.
   *
   * @todo Implement the initialization of 3D post-scan data.
   * @todo Implement the initialization of RF data.
   * @todo Make the method non-blocking (for usQtApplication).
   * @todo Throw an usGrabberException when the data initialization fails.
   */
  void start();

  usMotorSettings getMotorSettings() const {return m_motorSettings;}

  //void grabFrame( ImageType * imageToWrite);

  /**
   * Close input connection.
   */
  void stop();

 private:

  usTransducerSettings m_transducerSettings;
  usMotorSettings m_motorSettings;

  usGrabberCommunicationInformations m_communicationInormations;

  us::ImageType m_imageType;
};
#endif // _WIN32
#endif // US_GRABBER_ULTRASONIX_H
