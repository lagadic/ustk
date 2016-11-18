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
#ifndef _WIN32

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>

usGrabberUltrasonix::usGrabberUltrasonix() :
  m_communicationInormations()
{
  m_communicationInormations.m_voldata = NULL;
  m_communicationInormations.m_frmInf = NULL;
}


usGrabberUltrasonix::~usGrabberUltrasonix()
{
  if(m_communicationInormations.m_voldata) free(m_communicationInormations.m_voldata);
  if(m_communicationInormations.m_frmInf) free(m_communicationInormations.m_frmInf);
}


void usGrabberUltrasonix::start()
{
  m_communicationInormations.m_iter = 0;
  m_communicationInormations.m_frmIdx = 0;
  m_communicationInormations.m_previousFrmIdx = 0;

  // Create communication endpoint.
#ifdef TCP
  m_communicationInormations.m_serv_fd = socket(AF_INET, SOCK_STREAM, 0);
#elif defined UDP
  m_communicationInormations.m_serv_fd = socket(AF_INET, SOCK_DGRAM, 0);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  if (m_communicationInormations.m_serv_fd < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "socket() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "socket() error");
  }

  // Set socket address.
  memset((char*)&m_communicationInormations.m_serv_addr, 0, sizeof(m_communicationInormations.m_serv_addr));
  m_communicationInormations.m_serv_addr.sin_family = AF_INET;
  m_communicationInormations.m_serv_addr.sin_addr.s_addr = INADDR_ANY;
  m_communicationInormations.m_serv_addr.sin_port = htons(PORT);

  // Bind.
  if (bind(m_communicationInormations.m_serv_fd, (struct sockaddr*)&m_communicationInormations.m_serv_addr, sizeof(m_communicationInormations.m_serv_addr)) < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "bind() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "bind() error");
  }
  std::cout << "bind() OK" << std::endl;

#ifdef TCP
  // Listen.
  if(listen(m_communicationInormations.m_serv_fd, 1) < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "listen() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "listen() error");
  }
  std::cout << "listen() OK" << std::endl;

  // Accept.
  std::cout << "Waiting for connection, please start sending data from the Ultrasonix device."
            << std::endl;
  m_communicationInormations.m_clilen = sizeof(m_communicationInormations.m_cli_addr);
  m_communicationInormations.m_cli_fd = accept(m_communicationInormations.m_serv_fd, (struct sockaddr *) &m_communicationInormations.m_cli_addr, &m_communicationInormations.m_clilen);
  if (m_communicationInormations.m_cli_fd < 0){
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "accept() failed (error " << errno << ")." << std::endl;
    throw vpException(vpException::ioError, "accept() error");
  }
#endif

  // Print client's IP
  printf("You got a connection from %s\n", inet_ntoa(m_communicationInormations.m_cli_addr.sin_addr));

  // Read .vol header.
  int szHeader = sizeof(m_communicationInormations.m_header);
  int n = 0;
  while (n < szHeader) {
#ifdef TCP
    ssize_t len = recv(m_communicationInormations.m_cli_fd, &m_communicationInormations.m_header + n, szHeader - n, 0);
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
    n += recvfrom(m_communicationInormations.m_serv_fd, &m_communicationInormations.m_header+n, szHeader-n, 0, (sockaddr*)&m_communicationInormations.m_cli_addr,
                  (socklen_t*)&m_communicationInormations.m_clilen);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  }

  n = 0;
  while (n < szHeader) {
#ifdef TCP
    ssize_t len = recv(m_communicationInormations.m_cli_fd, &m_communicationInormations.m_header + n, szHeader - n, 0);
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
    n += recvfrom(m_communicationInormations.m_serv_fd, &m_communicationInormations.m_header+n, szHeader-n, 0, (sockaddr*)&m_communicationInormations.m_cli_addr,
                  (socklen_t*)&m_communicationInormations.m_clilen);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
  }

  m_communicationInormations.m_localTime = vpTime::measureTimeMs();
  m_communicationInormations.m_deviceTime = m_communicationInormations.m_header.startTime;

  // Print data info
  std::cout << std::endl << "Data header information: " << std::endl
            << "   Ultrasonix time reference = " << m_communicationInormations.m_deviceTime << "ms" << std::endl
            << "   type = " << m_communicationInormations.m_header.imageType << std::endl
            << "   volumes = " << m_communicationInormations.m_header.volumeNumber << std::endl
            << "   fpv = " << m_communicationInormations.m_header.framesPerVolume << std::endl
            << "   w = " << m_communicationInormations.m_header.width << std::endl
            << "   h = " << m_communicationInormations.m_header.height << std::endl
            << "   ss = " << m_communicationInormations.m_header.sampleSize << std::endl
            << "   degPerFr = " << m_communicationInormations.m_header.degPerFr / 1000.0 << "°" << std::endl
            << "   BSampleFreq = " << m_communicationInormations.m_header.BSampleFreq << "Hz" << std::endl
            << "   ProbeElementPitch = " << m_communicationInormations.m_header.ProbeElementPitch << "µm" << std::endl
            << "   ProbeRadius = " << m_communicationInormations.m_header.ProbeRadius << "µm" << std::endl
            << "   MotorRadius = " << m_communicationInormations.m_header.MotorRadius << "µm" << std::endl
            << "   frameRate = " << m_communicationInormations.m_header.framerate << std::endl << std::endl;

  // Check data type
  if (m_communicationInormations.m_header.imageType == 0) {
    std::cout << "Pre-scan data mode." << std::endl;
    m_imageType = TYPE_PRESCAN;
    m_communicationInormations.m_szFrm = (m_communicationInormations.m_header.width * m_communicationInormations.m_header.height * (m_communicationInormations.m_header.sampleSize / 8));
  } else if (m_communicationInormations.m_header.imageType == 1) {
    std::cout << "Post-scan data mode" << std::endl;
    m_imageType = TYPE_POSTSCAN;
    m_communicationInormations.m_szFrm = m_communicationInormations.m_header.width * m_communicationInormations.m_header.height;
  } else if (m_communicationInormations.m_header.imageType == 2) {
    std::cout << "RF data mode" << std::endl;
    m_imageType = TYPE_RF;
    m_communicationInormations.m_szFrm = m_communicationInormations.m_header.width * m_communicationInormations.m_header.height * 2; // RF = short = 2 bytes
  } else {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
              << "Invalid volume data mode (" << m_communicationInormations.m_header.imageType << ")" << std::endl;
    exit(EXIT_FAILURE);
  }

  m_communicationInormations.m_szVol = m_communicationInormations.m_szFrm * m_communicationInormations.m_header.framesPerVolume;
  m_communicationInormations.m_voldata = (unsigned char*)malloc(m_communicationInormations.m_szVol * sizeof(unsigned char));

  // frmIdx + frmTime + frmData
  m_communicationInormations.m_byteSizeFrmInf = sizeof(int) + sizeof(double) + m_communicationInormations.m_szFrm * sizeof(unsigned char);

  m_communicationInormations.m_frmInf = (char*)malloc(m_communicationInormations.m_byteSizeFrmInf);

  // Skip first frames
  for (int i = 0; i < NUM_FRAME_OFFSET; ++i) {
    n = 0;
    while(n < m_communicationInormations.m_byteSizeFrmInf) {
#ifdef TCP
      ssize_t len = recv(m_communicationInormations.m_cli_fd, m_communicationInormations.m_frmInf + n, m_communicationInormations.m_byteSizeFrmInf - n, 0);
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
      ssize_t len = recvfrom(m_communicationInormations.m_serv_fd, m_communicationInormations.m_frmInf + n, m_communicationInormations.m_byteSizeFrmInf - n, 0,
                             (sockaddr*)&m_communicationInormations.m_cli_addr, (socklen_t*)&m_communicationInormations.m_clilen);
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
  if (m_communicationInormations.m_header.imageType == 0) // Prescan
  {

    if (m_communicationInormations.m_header.framesPerVolume == 1) // 2D
    {
      m_transducerSettings= usTransducerSettings (m_communicationInormations.m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_communicationInormations.m_header.width, true,US_4DC7_BSAMPLE_DISTANCE *  m_communicationInormations.m_header.height );
    }
    else // 3D
    {

      m_transducerSettings= usTransducerSettings (m_communicationInormations.m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_communicationInormations.m_header.width, true,US_4DC7_BSAMPLE_DISTANCE *  m_communicationInormations.m_header.height );

      m_motorSettings= usMotorSettings (m_communicationInormations.m_header.MotorRadius / 1000000.0,vpMath::rad(m_communicationInormations.m_header.degPerFr / 1000.0),
                                        m_communicationInormations.m_header.framesPerVolume, usMotorSettings::TiltingMotor);
    }
  }
  else if (m_communicationInormations.m_header.imageType == 1) // Postscan
  {
    std::cout << "fpv : " << m_communicationInormations.m_header.framesPerVolume << std::endl;
    if (m_communicationInormations.m_header.framesPerVolume == 1) // 2D
    {
      /*std::cerr << "Error: in in usGrabberUltrasonix::start(): "
                << "Handling of 2D postscan data is not implemented." << std::endl;
      exit(EXIT_FAILURE);*/

      m_transducerSettings= usTransducerSettings (m_communicationInormations.m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_communicationInormations.m_header.width, true,US_4DC7_BSAMPLE_DISTANCE *  m_communicationInormations.m_header.height );

      /*m_data = new usImagePostScan2D();
      dynamic_cast<usDataPostscan2D*>(m_data)->resize(m_header.h, m_header.w);
      m_data->setMode(POSTSCAN_2D);
      m_data->setScannerType(SONIX_TOUCH);
      m_data->setProbeType(US_4DC7);*/
    }
    else // 3D
    {
      /*std::cerr << "Error: in in usGrabberUltrasonix::start(): "
                << "Handling of 3D postscan data is not implemented." << std::endl;
      exit(EXIT_FAILURE);*/

      m_transducerSettings= usTransducerSettings (m_communicationInormations.m_header.ProbeRadius / 1000000.0, vpMath::rad(US_4DC7_DEG_PER_LINE),
                                                  m_communicationInormations.m_header.width, true,US_4DC7_BSAMPLE_DISTANCE *  m_communicationInormations.m_header.height );

      m_motorSettings= usMotorSettings (m_communicationInormations.m_header.MotorRadius / 1000000.0,vpMath::rad(m_communicationInormations.m_header.degPerFr / 1000.0),1,
                                        usMotorSettings::TiltingMotor);
    }
  }
  else if (m_communicationInormations.m_header.imageType == 2) // RF
  {
    std::cerr << "Error: in in usGrabberUltrasonix::start(): "
              << "Handling of data type " << m_communicationInormations.m_header.imageType << " (RF) is not implemented." << std::endl;
    exit(EXIT_FAILURE);
  }
  else // Wrong data type
  {
    std::cerr << "Error: in in usGrabberUltrasonix::start(): "
              << "Wrong data type: " << m_communicationInormations.m_header.imageType << std::endl;
    exit(EXIT_FAILURE);
  }

  m_communicationInormations.m_frameRate = m_communicationInormations.m_header.framerate;

  std::cout << "Grabber initialized." << std::endl;
}


void usGrabberUltrasonix::stop() {
  // close client socket
#ifdef TCP
  close(m_communicationInormations.m_cli_fd);
  close(m_communicationInormations.m_serv_fd);
#elif defined UDP
  close(m_communicationInormations.m_serv_fd);
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif

  // free
  free(m_communicationInormations.m_frmInf);
  m_communicationInormations.m_frmInf = NULL;
}


int usGrabberUltrasonix::getSocket() { return m_communicationInormations.m_cli_fd; }


double usGrabberUltrasonix::getFrameRate() { return m_communicationInormations.m_frameRate; }

#endif
