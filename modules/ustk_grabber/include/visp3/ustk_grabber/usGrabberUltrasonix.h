/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#ifndef US_GRABBER_ULTRASONIX_H
#define US_GRABBER_ULTRASONIX_H

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
 *
 * Class for Ultrasonix data grabber.
 */
class VISP_EXPORT usGrabberUltrasonix {
 public:
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


  struct usGrabberCommunicationInformations
  {
  int m_serv_fd;
  int m_cli_fd;
#ifdef TCP
  socklen_t m_clilen;
#endif // TCP
#ifdef UDP
  int m_clilen;
#endif // UDP
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
  //Enum to return the grabbed image type
  //It's known only in start(), when we read the header sent from the ultrasonix station.
  typedef enum{
    TYPE_UNKNOWN = -1,
    TYPE_RF,
    TYPE_PRESCAN,
    TYPE_POSTSCAN
  } UsGrabberUltrasonixImageType;
  /**
   * Constructor
   */
  usGrabberUltrasonix();

  /**
   * Destructor.
   */
   ~usGrabberUltrasonix();

  /**
   * Initialization method.
   *
   * The method performs the following tasks:
   *  - Connection to the Ultrasonix scanner via TCP.
   *    The methods blocks until a connection is established.
   *  - If the connection is successful, initialization of the usData container with the corresponding type.
   *    This will consume the first frame sent by the scanner to determine the data type.
   *    Currently supported types are 2D and 3D prescan, and 2D postcan.
   *
   * @todo Implement the initialization of 3D postscan data.
   * @todo Implement the initialization of RF data.
   * @todo Make the method non-blocking (for usQtApplication).
   * @todo Throw an usGrabberException when the data initialization fails.
   */
  void start();

  usGrabberCommunicationInformations* getCommunicationsInformations(){return &m_communicationInormations;}

  UsGrabberUltrasonixImageType getImageType() const {return m_imageType;}

  usTransducerSettings getTransducerSettings() const {return m_transducerSettings;}

  usMotorSettings getMotorSettings() const {return m_motorSettings;}

  //void grabFrame( ImageType * imageToWrite);

  /**
   * Close input connection.
   */
  void stop();

  /**
   * Get the socket identifier.
   */
  int getSocket();

  /**
   * Get the frame rate.
   */
  double getFrameRate();
 private:

  usTransducerSettings m_transducerSettings;
  usMotorSettings m_motorSettings;

  usGrabberCommunicationInformations m_communicationInormations;

  UsGrabberUltrasonixImageType m_imageType;
};
#endif // _WIN32
#endif // US_GRABBER_ULTRASONIX_H
