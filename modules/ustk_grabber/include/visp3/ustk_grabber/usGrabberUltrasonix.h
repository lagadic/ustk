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

#include <visp3/ustk_core/usTransducerSettings.h>
#include <visp3/ustk_core/usMotorSettings.h>

//usImages
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

#define TCP
#define PORT 5555
#define NUM_FRAME_OFFSET 2

#ifndef M_PI
#define M_PI 4.0 * atan(1.0)
#endif

#ifndef DEG2RAD
#define DEG2RAD(a) (a * M_PI / 180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG(a) (a * 180.0 / M_PI)
#endif

// 4DC7-3 probe properties
#define US_4DC7_DEG_PER_LINE 0.608767657441f
//#define US_4DC7_PROBE_RADIUS 0.04f // m
//#define US_4DC7_MOTOR_RADIUS 0.02725f // m
#define US_4DC7_BSAMPLE_DISTANCE 0.000308f // m

struct SocketHeader3D
{
  double startTime;
  int type;
  int volumes;
  int fpv;
  int w;
  int h;
  int ss;
  int degPerFr;
  int BSampleFreq; // hz
  int ProbeElementPitch; //micron
  int ProbeRadius; //micron
  int MotorRadius; //micron
  int framerate;
};

/**
 * @class usGrabberUltrasonix
 * @brief Ultrasonix data grabber.
 *
 * Class for Ultrasonix data grabber.
 */
template <class ImageType>
class usGrabberUltrasonix {
 public:
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

  /**
   * Grab new data.
   *
   * The method reads one data instance from the TCP connection and updates the contents of the usData object.
   * Currently supported types are 2D and 3D prescan, and 2D postcan.
   *
   * @return true if there is new data available.
   *
   * @todo Implement grabbing of 3D postscan data.
   * @todo Implement grabbing of RF data.
   * @todo Throw an usGrabberException when the data grabbing fails.
   */
  void grab();

  void grabFrame( ImageType * imageToWrite);

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

  ImageType *m_usImage;
  usTransducerSettings m_transducerSettings;
  usMotorSettings m_motorSettings;
};

template <class Type>
usGrabberUltrasonix<Type>::usGrabberUltrasonix() :
  m_voldata(NULL),
  m_frmInf(NULL)
{

}

template <class Type>
usGrabberUltrasonix<Type>::~usGrabberUltrasonix()
{
  if(m_voldata) free(m_voldata);
  if(m_frmInf) free(m_frmInf);
}

template <class Type>
void usGrabberUltrasonix<Type>::start()
{
  m_iter = 0;
  m_frmIdx = 0;
  m_previousFrmIdx = 0;

  // Create communication endpoint.
#ifdef TCP
  m_serv_fd = socket(AF_INET, SOCK_STREAM, 0);
#elif defined UDP
  m_serv_fd = socket(AF_INET, SOCK_DGRAM, 0);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  if (m_serv_fd < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "socket() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "socket() error");
  }

  // Set socket address.
  memset((char*)&m_serv_addr, 0, sizeof(m_serv_addr));
  m_serv_addr.sin_family = AF_INET;
  m_serv_addr.sin_addr.s_addr = INADDR_ANY;
  m_serv_addr.sin_port = htons(PORT);

  // Bind.
  if (bind(m_serv_fd, (struct sockaddr*)&m_serv_addr, sizeof(m_serv_addr)) < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "bind() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "bind() error");
  }
  std::cout << "bind() OK" << std::endl;

#ifdef TCP
  // Listen.
  if(listen(m_serv_fd, 1) < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "listen() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "listen() error");
  }
  std::cout << "listen() OK" << std::endl;

  // Accept.
  std::cout << "Waiting for connection, please start sending data from the Ultrasonix device."
            << std::endl;
  m_clilen = sizeof(m_cli_addr);
  m_cli_fd = accept(m_serv_fd, (struct sockaddr *) &m_cli_addr, &m_clilen);
  if (m_cli_fd < 0){
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "accept() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "accept() error");
  }
#endif

  // Print client's IP
  printf("You got a connection from %s\n", inet_ntoa(m_cli_addr.sin_addr));

  // Read .vol header.
  int szHeader = sizeof(m_header);
  int n = 0;
  while (n < szHeader) {
#ifdef TCP
    ssize_t len = recv(m_cli_fd, &m_header + n, szHeader - n, 0);
    if (len == 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
                << "recv() returned value 0: "
                << "No messages are available to be received "
                << "and the peer has performed an orderly shutdown." << std::endl;
      throw vpException(vpException::ioError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
                << "recv() returned value " << len << std::endl;
      throw vpException(vpException::ioError, "recv() error");
    }
    n += len;
#elif defined UDP
    n += recvfrom(m_serv_fd, &m_header+n, szHeader-n, 0, (sockaddr*)&m_cli_addr,
                  (socklen_t*)&m_clilen);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  }

  n = 0;
  while (n < szHeader) {
#ifdef TCP
    ssize_t len = recv(m_cli_fd, &m_header + n, szHeader - n, 0);
    if (len == 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
                << "recv() returned value 0: "
                << "No messages are available to be received "
                << "and the peer has performed an orderly shutdown." << std::endl;
      throw vpException(vpException::ioError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
                << "recv() returned value " << len << std::endl;
      throw vpException(vpException::ioError, "recv() error");
    }
    n += len;
#elif defined UDP
    n += recvfrom(m_serv_fd, &m_header+n, szHeader-n, 0, (sockaddr*)&m_cli_addr,
                  (socklen_t*)&m_clilen);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  }

  m_localTime = vpTime::measureTimeMs();
  m_deviceTime = m_header.startTime;

  // Print data info
  std::cout << std::endl << "Data header information: " << std::endl
            << "   Ultrasonix time reference = " << m_deviceTime << "ms" << std::endl
            << "   type = " << m_header.type << std::endl
            << "   volumes = " << m_header.volumes << std::endl
            << "   fpv = " << m_header.fpv << std::endl
            << "   w = " << m_header.w << std::endl
            << "   h = " << m_header.h << std::endl
            << "   ss = " << m_header.ss << std::endl
            << "   degPerFr = " << m_header.degPerFr / 1000.0 << "°" << std::endl
            << "   BSampleFreq = " << m_header.BSampleFreq << "Hz" << std::endl
            << "   ProbeElementPitch = " << m_header.ProbeElementPitch << "µm" << std::endl
            << "   ProbeRadius = " << m_header.ProbeRadius << "µm" << std::endl
            << "   MotorRadius = " << m_header.MotorRadius << "µm" << std::endl
            << "   frameRate = " << m_header.framerate << std::endl << std::endl;

  // Check data type
  if (m_header.type == 0) {
    std::cout << "Pre-scan data mode." << std::endl;
    m_szFrm = (m_header.w * m_header.h * (m_header.ss / 8));
  } else if (m_header.type == 1) {
    std::cout << "Post-scan data mode" << std::endl;
    m_szFrm = m_header.w * m_header.h;
  } else if (m_header.type == 2) {
    std::cout << "RF data mode" << std::endl;
    m_szFrm = m_header.w * m_header.h * 2; // RF = short = 2 bytes
  } else {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "Invalid volume data mode (" << m_header.type << ")" << std::endl;
    exit(EXIT_FAILURE);
  }

  m_szVol = m_szFrm * m_header.fpv;
  m_voldata = (unsigned char*)malloc(m_szVol * sizeof(unsigned char));

  // frmIdx + frmTime + frmData
  m_byteSizeFrmInf = sizeof(int) + sizeof(double) + m_szFrm * sizeof(unsigned char);

  m_frmInf = (char*)malloc(m_byteSizeFrmInf);

  // Skip first frames
  for (int i = 0; i < NUM_FRAME_OFFSET; ++i) {
    n = 0;
    while(n < m_byteSizeFrmInf) {
#ifdef TCP
      ssize_t len = recv(m_cli_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0);
      if (len == 0) {
        std::cerr << "Error: in usGrabberUltrasonix::start(): "
                  << "recv() returned value 0: "
                  << "No messages are available to be received "
                  << "and the peer has performed an orderly shutdown." << std::endl;
        throw vpException(vpException::ioError, "Connection shutdown by peer.");
      }
      if (len < 0) {
        std::cerr << "Error: in usGrabberUltrasonix::start(): "
                  << "recv() returned value " << len << std::endl;
        throw vpException(vpException::ioError, "recv() error");
      }
      n += len;
#elif defined UDP
      ssize_t len = recvfrom(m_serv_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0,
                             (sockaddr*)&m_cli_addr, (socklen_t*)&m_clilen);
      if (len == 0) {
        std::cerr << "Error: in usGrabberUltrasonix::start(): "
                  << "recvfrom() returned value 0: "
                  << "No messages are available to be received "
                  << "and the peer has performed an orderly shutdown." << std::endl;
        throw vpException(vpException::ioError, "Connection shutdown by peer.");
      }
      if (len < 0) {
        std::cerr << "Error: in usGrabberUltrasonix::start(): "
                  << "recvfrom() returned value " << len << std::endl;
        throw vpException(vpException::ioError, "recv() error");
      }
      n += len;
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
    }
  }

  //init acquisition settings
  // Init data
  if (m_header.type == 0) // Prescan
  {

    if (m_header.fpv == 1) // 2D
    {
      m_transducerSettings= usTransducerSettings (m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_header.w, true,US_4DC7_BSAMPLE_DISTANCE *  m_header.h );
    }
    else // 3D
    {

      m_transducerSettings= usTransducerSettings (m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_header.w, true,US_4DC7_BSAMPLE_DISTANCE *  m_header.h );

      m_motorSettings= usMotorSettings (m_header.MotorRadius / 1000000.0,vpMath::rad(m_header.degPerFr / 1000.0),1,
                                        usMotorSettings::TiltingMotor);
    }
  }
  else if (m_header.type == 1) // Postscan
  {
    if (m_header.fpv == 1) // 2D
    {
      std::cerr << "Error: in in usGrabberUltrasonix::start(): "
                << "Handling of 2D postscan data is not implemented." << std::endl;
      exit(EXIT_FAILURE);

      m_transducerSettings= usTransducerSettings (m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_header.w, true,US_4DC7_BSAMPLE_DISTANCE *  m_header.h );

      /*m_data = new usImagePostScan2D();
      dynamic_cast<usDataPostscan2D*>(m_data)->resize(m_header.h, m_header.w);
      m_data->setMode(POSTSCAN_2D);
      m_data->setScannerType(SONIX_TOUCH);
      m_data->setProbeType(US_4DC7);*/
    }
    else // 3D
    {
      std::cerr << "Error: in in usGrabberUltrasonix::start(): "
                << "Handling of 3D postscan data is not implemented." << std::endl;
      exit(EXIT_FAILURE);

      m_transducerSettings= usTransducerSettings (m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_header.w, true,US_4DC7_BSAMPLE_DISTANCE *  m_header.h );

      m_motorSettings= usMotorSettings (m_header.MotorRadius / 1000000.0,vpMath::rad(m_header.degPerFr / 1000.0),1,
                                        usMotorSettings::TiltingMotor);
    }
  }
  else if (m_header.type == 2) // RF
  {
    std::cerr << "Error: in in usGrabberUltrasonix::start(): "
              << "Handling of data type " << m_header.type << " (RF) is not implemented." << std::endl;
    exit(EXIT_FAILURE);
  }
  else // Wrong data type
  {
    std::cerr << "Error: in in usGrabberUltrasonix::start(): "
              << "Wrong data type: " << m_header.type << std::endl;
    exit(EXIT_FAILURE);
  }

  m_frameRate = m_header.framerate;

  std::cout << "Grabber initialized." << std::endl;
}
/*
void usGrabberUltrasonix::grab() {
  // Grab a volume
  for(int i = 0; i < m_header.fpv; ++i) {
    int n = 0;

    // Grab frame information
    while(n < m_byteSizeFrmInf){
#ifdef TCP
      ssize_t len = recv(m_cli_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0);
      if (len == 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recv() returned value 0: "
      << "No messages are available to be received "
      << "and the peer has performed an orderly shutdown." << std::endl;
        throw vpException(vpException::ioError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recv() returned value " << len << std::endl;
  throw vpException(vpException::ioError, "recv() error");
      }
      n += len;
#elif defined UDP
      ssize_t len = recvfrom(m_serv_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0,
        (sockaddr*)&m_cli_addr, (socklen_t*)&m_clilen);
      if (len == 0) {setMotorSettings
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recvfrom() returned value 0: "
      << "No messages are available to be received "
      << "and the peer has performed an orderly shutdown." << std::endl;
        throw vpException(vpException::ioError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recvfrom() returned value " << len << std::endl;
  throw vpException(vpException::ioError, "recvfrom() error");
      }
      n += len;
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
      //std::cout << "Received " << n << " bytes / " << m_byteSizeFrmInf << std::endl;
    }

    // Parse frame information
    memcpy(&m_totFrmIdx, m_frmInf, sizeof(int));
    memcpy(&m_frmTime, m_frmInf + sizeof(int), sizeof(double));

    if(m_iter == 0)
      m_previousFrmIdx = m_totFrmIdx - 1;

    if(m_totFrmIdx != m_previousFrmIdx + 1) {
      throw vpException(vpException::ioError,
             "Error in data index: totFrmIdx = %d; previousFrmIdx = %d",
             m_totFrmIdx, m_previousFrmIdx);
    }

    m_previousFrmIdx = m_totFrmIdx;

    // offset
    m_totFrmIdx -= NUM_FRAME_OFFSET;
    m_frmIdx = m_totFrmIdx % m_header.fpv;

    // Copy data
    memcpy(m_voldata + m_frmIdx * m_szFrm,  m_frmInf + sizeof(int) + sizeof(double),
     m_szFrm * sizeof(unsigned char));
  }

  // Update data
  if (m_data->getMode() == PRESCAN_2D)
    {
      usDataPrescan2D *data_d = dynamic_cast<usDataPrescan2D*>(m_data);

      for (unsigned int i = 0; i < data_d->getHeight(); ++i)
  for (unsigned int j = 0; j < data_d->getWidth(); ++j)
    (*data_d)(i, j, m_voldata[i + j * data_d->getHeight()]);
    }
  else if (m_data->getMode() == PRESCAN_3D)
    {
      memcpy(dynamic_cast<usDataPrescan3D*>(m_data)->getData(), m_voldata,
       m_szVol * sizeof(unsigned char));
    }
  else if (m_data->getMode() == POSTSCAN_2D)
    {
      usDataPostscan2D *data_d = dynamic_cast<usDataPostscan2D*>(m_data);
      for (unsigned int i = 0; i < data_d->getHeight(); ++i)
  for (unsigned int j = 0; j < data_d->getWidth(); ++j)
    (*data_d)(i, j, m_voldata[j + i * data_d->getWidth()]);
    }
  else if (m_data->getMode() == POSTSCAN_3D)
    {
      std::cerr << "Error: in in usGrabberUltrasonix::grab(): "
    << "Handling of 3D postscan data is not implemented." << std::endl;
      exit(EXIT_FAILURE);
    }
  else
    {
      std::cerr << "Error: in in usGrabberUltrasonix::grab(): "
    << "This data mode is not supported." << std::endl;
      exit(EXIT_FAILURE);
    }

  // Set timestamp and data index
  m_data->setTimestamp(m_frmTime - m_deviceTime + m_localTime);
  m_data->setDataIdx(static_cast<int>(m_totFrmIdx / static_cast<float>(m_header.fpv)));

  m_iter++;
}*/

template <class Type>
void usGrabberUltrasonix<Type>::grabFrame(Type* imageToWrite) {
  // Grab a single frame
  int n = 0;
  m_usImage = imageToWrite;
  // Grab frame information
  while(n < m_byteSizeFrmInf){
#ifdef TCP
    ssize_t len = recv(m_cli_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0);
    if (len == 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grabFrame(): "
                << "recv() returned value 0: "
                << "No messages are available to be received "
                << "and the peer has performed an orderly shutdown." << std::endl;
      throw vpException(vpException::ioError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grabFrame(): "
                << "recv() returned value " << len << std::endl;
      throw vpException(vpException::ioError, "recv() error");
    }
    n += len;
#elif defined UDP
    ssize_t len = recvfrom(m_serv_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0,
                           (sockaddr*)&m_cli_addr, (socklen_t*)&m_clilen);
    if (len == 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grabFrame(): "
                << "recvfrom() returned value 0: "
                << "No messages are available to be received "
                << "and the peer has performed an orderly shutdown." << std::endl;
      throw vpException(vpException::ioError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grabFrame(): "
                << "recvfrom() returned value " << len << std::endl;
      throw vpException(vpException::ioError, "recvfrom() error");
    }
    n += len;
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
    //std::cout << "Received " << n << " bytes / " << m_byteSizeFrmInf << std::endl;
  }

  // Parse frame information
  memcpy(&m_totFrmIdx, m_frmInf, sizeof(int));
  memcpy(&m_frmTime, m_frmInf + sizeof(int), sizeof(double));

  if(m_iter == 0)
    m_previousFrmIdx = m_totFrmIdx - 1;

  if(m_totFrmIdx != m_previousFrmIdx + 1) {
    throw vpException(vpException::ioError,
                      "Error in data index: totFrmIdx = %d; previousFrmIdx = %d",
                      m_totFrmIdx, m_previousFrmIdx);
  }

  m_previousFrmIdx = m_totFrmIdx;

  // offset
  m_totFrmIdx -= NUM_FRAME_OFFSET;
  m_frmIdx = m_totFrmIdx % m_header.fpv;

  // Copy data
  memcpy(m_voldata + m_frmIdx * m_szFrm,  m_frmInf + sizeof(int) + sizeof(double),
         m_szFrm * sizeof(unsigned char));

  // Update data
  if (m_header.type == 0 && m_header.fpv == 1) // pre-scan 2D
  {
    //usImagePreScan2D *data_d = dynamic_cast<usImagePreScan2D*>(m_data);
    /*
  memcpy(dynamic_cast<usDataPrescan2D*>(m_data)->bitmap, m_voldata,
  m_szVol * sizeof(unsigned char));
      */
    for (unsigned int i = 0; i < m_header.h; ++i) {
      for (unsigned int j = 0; j < m_header.w; ++j) {
        (*m_usImage)(i, j, m_voldata[i + j * m_header.h]);
      }
    }
    m_usImage->setTransducerSettings(m_transducerSettings);
  }
  else if (m_header.type == 0 && m_header.fpv != 1) //prescan 3D
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "Handling of 3D prescan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);
    /*memcpy(dynamic_cast<usDataPrescan3D*>(m_data)->getData() + m_frmIdx * m_szFrm,
           m_voldata + m_frmIdx * m_szFrm,
           m_szFrm * sizeof(unsigned char));*/
    //update image settings
    m_usImage->setTransducerSettings(m_transducerSettings);
    //m_usImage->setMotorSettings(m_motorSettings);
  }
  else if (m_header.type == 1 && m_header.fpv == 1) //postScan 2D
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "Handling of 2D postscan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);
    /*usDataPostscan2D *data_d = dynamic_cast<usDataPostscan2D*>(m_data);
    for (unsigned int i = 0; i < data_d->getHeight(); ++i)
      for (unsigned int j = 0; j < data_d->getWidth(); ++j)
        (*data_d)(i, j, m_voldata[j + i * data_d->getWidth()]);*/
    m_usImage->setTransducerSettings(m_transducerSettings);
  }
  else if (m_header.type == 1 && m_header.fpv != 1)
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "Handling of 3D postscan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);

    m_usImage->setTransducerSettings(m_transducerSettings);
    //m_usImage->setMotorSettings(m_motorSettings);
  }
  else
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "This data mode is not supported." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Set timestamp and data index
  /*m_data->setTimestamp(m_frmTime - m_deviceTime + m_localTime);
  m_data->setDataIdx(static_cast<int>(m_totFrmIdx));*/
  m_iter++;
}

template <class Type>
void usGrabberUltrasonix<Type>::stop() {
  // close client socket
#ifdef TCP
  close(m_cli_fd);
  close(m_serv_fd);
#elif defined UDP
  close(m_serv_fd);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif

  // free
  free(m_frmInf);
  m_frmInf = NULL;
}

template <class Type>
int usGrabberUltrasonix<Type>::getSocket() { return m_cli_fd; }

template <class Type>
double usGrabberUltrasonix<Type>::getFrameRate() { return m_frameRate; }

#endif // US_GRABBER_ULTRASONIX_H
