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

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <unistd.h> // for close(int socket_id)
#include <cerrno>
#include <visp/vpTime.h>
#include <UsTk/usDataPrescan2D.h>
#include <UsTk/usDataPrescan3D.h>
#include <UsTk/usDataPostscan2D.h>
#include <UsTk/usGrabberException.h>

usGrabberUltrasonix::usGrabberUltrasonix() :
    m_voldata(NULL),
    m_frmInf(NULL)
{

}

usGrabberUltrasonix::~usGrabberUltrasonix()
{
    if(m_voldata) free(m_voldata);
    if(m_frmInf) free(m_frmInf);
}

void usGrabberUltrasonix::start()
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
    throw usGrabberException(usGrabberException::socketError, "socket() error");
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
    throw usGrabberException(usGrabberException::bindError, "bind() error");
  }
  std::cout << "bind() OK" << std::endl;

#ifdef TCP
  // Listen.
  if(listen(m_serv_fd, 1) < 0) {
    std::cerr << "Error: in usGrabberUltrasonix::start(): "
        << "listen() failed (error " << errno << ")." << std::endl;
    throw usGrabberException(usGrabberException::listenError, "listen() error");
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
    throw usGrabberException(usGrabberException::acceptError, "accept() error");
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
      throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
    << "recv() returned value " << len << std::endl;
      throw usGrabberException(usGrabberException::recvError, "recv() error");
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
      throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::start(): "
    << "recv() returned value " << len << std::endl;
      throw usGrabberException(usGrabberException::recvError, "recv() error");
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
  throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::start(): "
      << "recv() returned value " << len << std::endl;
  throw usGrabberException(usGrabberException::recvError, "recv() error");
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
  throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::start(): "
      << "recvfrom() returned value " << len << std::endl;
  throw usGrabberException(usGrabberException::recvError, "recv() error");
      }
      n += len;
#else
#error "No connection protocol defined. Please define either TCP or UDP."
#endif
    }
  }

  // Init data
  if (m_header.type == 0) // Prescan
    {
      float probeRadius = m_header.ProbeRadius / 1000000.0; // µm -> m
      float motorRadius = m_header.MotorRadius / 1000000.0; // µm -> m
      float probeElementPitch = m_header.ProbeElementPitch / 1000000.0; // µm -> m
      float lineAngle = DEG2RAD(US_4DC7_DEG_PER_LINE);
      float frameAngle = DEG2RAD(m_header.degPerFr / 1000.0);

      if (m_header.fpv == 1) // 2D
  {
    m_data = new usDataPrescan2D(m_header.h, m_header.w, probeRadius, lineAngle,
               US_4DC7_BSAMPLE_DISTANCE,
               m_header.BSampleFreq, probeElementPitch);
    m_data->setScannerType(SONIX_TOUCH);
    m_data->setProbeType(US_4DC7);
  }
      else // 3D
  {
    m_data = new usDataPrescan3D(m_header.h, m_header.w, m_header.fpv,
               probeRadius, motorRadius,
               lineAngle, frameAngle, US_4DC7_BSAMPLE_DISTANCE,
               m_header.BSampleFreq, probeElementPitch);
    m_data->setScannerType(SONIX_TOUCH);
    m_data->setProbeType(US_4DC7);
  }
    }
  else if (m_header.type == 1) // Postscan
    {
      if (m_header.fpv == 1) // 2D
  {
    m_data = new usDataPostscan2D();
    dynamic_cast<usDataPostscan2D*>(m_data)->resize(m_header.h, m_header.w);
    m_data->setMode(POSTSCAN_2D);
    m_data->setScannerType(SONIX_TOUCH);
    m_data->setProbeType(US_4DC7);
  }
      else // 3D
  {
    std::cerr << "Error: in in usGrabberUltrasonix::start(): "
        << "Handling of 3D postscan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);
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
        throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recv() returned value " << len << std::endl;
  throw usGrabberException(usGrabberException::recvError, "recv() error");
      }
      n += len;
#elif defined UDP
      ssize_t len = recvfrom(m_serv_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0,
        (sockaddr*)&m_cli_addr, (socklen_t*)&m_clilen);
      if (len == 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recvfrom() returned value 0: "
      << "No messages are available to be received "
      << "and the peer has performed an orderly shutdown." << std::endl;
        throw usGrabberException(usGrabberException::recvfromError, "Connection shutdown by peer.");
      }
      if (len < 0) {
  std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recvfrom() returned value " << len << std::endl;
  throw usGrabberException(usGrabberException::recvfromError, "recvfrom() error");
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
      throw usGrabberException(usGrabberException::indexError,
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
      /*
  memcpy(dynamic_cast<usDataPrescan2D*>(m_data)->bitmap, m_voldata,
  m_szVol * sizeof(unsigned char));
      */
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
}

void usGrabberUltrasonix::grabFrame() {
  // Grab a single frame
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
      throw usGrabberException(usGrabberException::recvError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grab(): "
      << "recv() returned value " << len << std::endl;
      throw usGrabberException(usGrabberException::recvError, "recv() error");
    }
    n += len;
#elif defined UDP
    ssize_t len = recvfrom(m_serv_fd, m_frmInf + n, m_byteSizeFrmInf - n, 0,
         (sockaddr*)&m_cli_addr, (socklen_t*)&m_clilen);
    if (len == 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grab(): "
    << "recvfrom() returned value 0: "
    << "No messages are available to be received "
    << "and the peer has performed an orderly shutdown." << std::endl;
      throw usGrabberException(usGrabberException::recvfromError, "Connection shutdown by peer.");
    }
    if (len < 0) {
      std::cerr << "Error: in usGrabberUltrasonix::grab(): "
    << "recvfrom() returned value " << len << std::endl;
  throw usGrabberException(usGrabberException::recvfromError, "recvfrom() error");
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
    throw usGrabberException(usGrabberException::indexError,
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
  if (m_data->getMode() == PRESCAN_2D)
    {
      usDataPrescan2D *data_d = dynamic_cast<usDataPrescan2D*>(m_data);
      /*
  memcpy(dynamic_cast<usDataPrescan2D*>(m_data)->bitmap, m_voldata,
  m_szVol * sizeof(unsigned char));
      */
      for (unsigned int i = 0; i < data_d->getHeight(); ++i)
  for (unsigned int j = 0; j < data_d->getWidth(); ++j)
    (*data_d)(i, j, m_voldata[i + j * data_d->getHeight()]);
    }
  else if (m_data->getMode() == PRESCAN_3D)
    {
      memcpy(dynamic_cast<usDataPrescan3D*>(m_data)->getData() + m_frmIdx * m_szFrm,
       m_voldata + m_frmIdx * m_szFrm,
       m_szFrm * sizeof(unsigned char));
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
  m_data->setDataIdx(static_cast<int>(m_totFrmIdx));

  m_iter++;
}

void usGrabberUltrasonix::stop() {
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

int usGrabberUltrasonix::getSocket() { return m_cli_fd; }

double usGrabberUltrasonix::getFrameRate() { return m_frameRate; }
