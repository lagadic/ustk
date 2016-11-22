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

#ifndef US_GRABBER_FRAME_H
#define US_GRABBER_FRAME_H

#ifndef _WIN32

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>

/**
 * @class usGrabberFrame
 * @brief Frame data grabber.
 * @ingroup module_ustk_grabber
 *
 * Template class to grab frames from ultrasonix station. The initialisation is done in usGrabberUltrasonix,
 * and the frame grabbing is done by the method grabFrame from this class.
 */
template <class ImageType>
class usGrabberFrame {
 public:
  /**
   * Constructor
   */
  usGrabberFrame();

  /**
   * Destructor.
   */
   ~usGrabberFrame();

  void grabFrame(ImageType * imageToWrite);

  void setCommunicationInformation(usGrabberUltrasonix::usGrabberCommunicationInformations *informations);

  void setTransducerSettings(usTransducerSettings transducerSettings);

  void grabFrame(ImageType* imageToWrite) ;

 private:
  usGrabberUltrasonix::usGrabberCommunicationInformations *m_informations;

  usTransducerSettings m_transducerSettings;
};



template <class ImageType>
usGrabberFrame<ImageType>::usGrabberFrame()
{

}

template <class ImageType>
usGrabberFrame<ImageType>::~usGrabberFrame()
{

}

template <class ImageType>
void usGrabberFrame<ImageType>::setCommunicationInformation(usGrabberUltrasonix::usGrabberCommunicationInformations *informations)
{
  m_informations = informations;
}

template <class ImageType>
void usGrabberFrame<ImageType>::setTransducerSettings(usTransducerSettings transducerSettings)
{
  m_transducerSettings = transducerSettings;
}


//Generic method (works for 2D)
template <class ImageType>
void usGrabberFrame<ImageType>::grabFrame(ImageType* imageToWrite) {
  // Grab a single frame
  int n = 0;
  // Grab frame information
  while(n < m_informations->m_byteSizeFrmInf){
#ifdef TCP
    ssize_t len = recv(m_informations->m_cli_fd, m_informations->m_frmInf + n, m_informations->m_byteSizeFrmInf - n, 0);
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
    ssize_t len = recvfrom(m_informations->m_serv_fd, m_informations->m_frmInf + n, m_informationsm_byteSizeFrmInf - n, 0,
                           (sockaddr*)&m_informationsm_cli_addr, (socklen_t*)&m_informationsm_clilen);
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
    //std::cout << "Received " << n << " bytes / " << m_informations->m_byteSizeFrmInf << std::endl;
  }

  // Parse frame information
  memcpy(&m_informations->m_totFrmIdx, m_informations->m_frmInf, sizeof(int));
  memcpy(&m_informations->m_frmTime, m_informations->m_frmInf + sizeof(int), sizeof(double));

  if(m_informations->m_iter == 0)
    m_informations->m_previousFrmIdx = m_informations->m_totFrmIdx - 1;

  if(m_informations->m_totFrmIdx != m_informations->m_previousFrmIdx + 1) {
    throw vpException(vpException::ioError,
                      "Error in data index: totFrmIdx = %d; previousFrmIdx = %d",
                      m_informations->m_totFrmIdx, m_informations->m_previousFrmIdx);
  }

  m_informations->m_previousFrmIdx = m_informations->m_totFrmIdx;

  // offset
  m_informations->m_totFrmIdx -= NUM_FRAME_OFFSET;
  m_informations->m_frmIdx = m_informations->m_totFrmIdx % m_informations->m_header.framesPerVolume;

  // Copy data
  memcpy(m_informations->m_voldata + m_informations->m_frmIdx * m_informations->m_szFrm,  m_informations->m_frmInf + sizeof(int) + sizeof(double),
         m_informations->m_szFrm * sizeof(unsigned char));

  // Update data
  if (m_informations->m_header.imageType == 0 && m_informations->m_header.framesPerVolume == 1) // pre-scan 2D
  {
    //usImagePreScan2D *data_d = dynamic_cast<usImagePreScan2D*>(m_data);
    /*
  memcpy(dynamic_cast<usDataPrescan2D*>(m_informations->m_data)->bitmap, m_informations->m_voldata,
  m_informations->m_szVol * sizeof(unsigned char));
      */
    std::cout << "pre scan 2D grabbing" << std::endl;
    for (int i = 0; i < m_informations->m_header.height; ++i) {
      for (int j = 0; j < m_informations->m_header.width; ++j) {
        (*imageToWrite)(i, j, m_informations->m_voldata[i + j * m_informations->m_header.height]);
      }
    }
    imageToWrite->setTransducerSettings(m_transducerSettings);
  }
  else if (m_informations->m_header.imageType == 0 && m_informations->m_header.framesPerVolume >3) //prescan 3D
  {
    /*std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "Handling of 3D pre-scan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);*/
    /*memcpy(dynamic_cast<usDataPrescan3D*>(m_data)->getData() + m_frmIdx * m_szFrm,
           m_voldata + m_frmIdx * m_szFrm,
           m_szFrm * sizeof(unsigned char));*/

    //Trying to send frame by frame in preScan2D format
    for (int i = 0; i < m_informations->m_header.height; ++i) {
      for (int j = 0; j < m_informations->m_header.width; ++j) {
        (*imageToWrite)(i, j, m_informations->m_voldata[m_informations->m_frmIdx * m_informations->m_szFrm + i + j * m_informations->m_header.height]);
      }
    }
    imageToWrite->setTransducerSettings(m_transducerSettings);
  }
  else if (m_informations->m_header.imageType == 1 && m_informations->m_header.framesPerVolume == 3) //postScan 2D
  {
    for (int i = 0; i < m_informations->m_header.height; ++i) {
      for (int j = 0; j < m_informations->m_header.width; ++j) {
        (*imageToWrite)(i, j, m_informations->m_voldata[i + j * m_informations->m_header.height]);
      }
    }
    imageToWrite->setTransducerSettings(m_transducerSettings);
  }
  else if (m_informations->m_header.imageType == 1 && m_informations->m_header.framesPerVolume != 1)
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "Handling of 3D post-scan data is not implemented." << std::endl;
    exit(EXIT_FAILURE);

    imageToWrite->setTransducerSettings(m_transducerSettings);
    //imageToWrite->setMotorSettings(m_informations->m_motorSettings);
  }
  else
  {
    std::cerr << "Error: in in usGrabberUltrasonix::grabFrame(): "
              << "This data mode is not supported." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Set timestamp and data index
  /*m_informations->m_data->setTimestamp(m_informations->m_frmTime - m_informations->m_deviceTime + m_informations->m_localTime);
  m_informations->m_data->setDataIdx(static_cast<int>(m_informations->m_totFrmIdx));*/
  m_informations->m_iter++;
}

#endif // _WIN32
#endif // US_GRABBER_FRAME_H
