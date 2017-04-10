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
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>

#if defined(USTK_GRABBER_HAVE_QT5)

#include <QDataStream>

/**
* Constructor. Inititializes the image, and manages Qt signal.
*/
usNetworkGrabberPreScan::usNetworkGrabberPreScan(usNetworkGrabber *parent) :
  usNetworkGrabber(parent)
{
  m_grabbedImage.init(0,0);
  connect(m_tcpSocket ,SIGNAL(readyRead()),this, SLOT(dataArrived()));
}


/**
* Destructor.
*/
usNetworkGrabberPreScan::~usNetworkGrabberPreScan()
{

}

/**
* Slot called when data is coming on the network.
* Manages the type of data is coming and read it. Emits newFrameArrived signal when a whole frame is available.
*/
// This function is called when the data is fully arrived from the server to the client
void usNetworkGrabberPreScan::dataArrived()
{
  ////////////////// HEADER READING //////////////////
  QDataStream in;
  in.setDevice(m_tcpSocket);
#if (defined(USTK_GRABBER_HAVE_QT5))
  in.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  in.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  int headerType;
  if(m_bytesLeftToRead == 0 ) { // do not try to read a header if last frame was not complete
    in >> headerType;
    if(m_verbose)
      std::cout << "header received, type = " << headerType << std::endl;
  }
  else {
    headerType = 0; // not a header received, but a part of a frame
  }
  //init confirm header received
  if(headerType == m_confirmHeader.headerId) {
    //read whole header
    in >> m_confirmHeader.initOk;
    in >> m_confirmHeader.probeId;

    if(m_confirmHeader.initOk == 0) {
      m_tcpSocket->close();
      throw(vpException(vpException::fatalError, "porta initialisation error, closing connection."));
    }
    if(m_verbose)
      std::cout << "porta init sucess, detected probe id = " << m_confirmHeader.probeId << std::endl;
  }

  //image header received
  else if(headerType == m_confirmHeader.headerId) {
    //read whole header
    in >> m_imageHeader.frameCount;
    in >> m_imageHeader.timeStamp;
    in >> m_imageHeader.dataLength;
    in >> m_imageHeader.ss;
    in >> m_imageHeader.imageType;
    in >> m_imageHeader.frameWidth;
    in >> m_imageHeader.frameHeight;
    in >> m_imageHeader.transducerRadius;
    in >> m_imageHeader.scanLinePitch;
    in >> m_imageHeader.scanLineNumber;
    in >> m_imageHeader.degPerFr;
    in >> m_imageHeader.framesPerVolume;

    if(m_verbose) {
      std::cout << "frameCount = " <<  m_imageHeader.frameCount << std::endl;
      std::cout << "timeStamp = " <<  m_imageHeader.timeStamp << std::endl;
      std::cout << "dataLength = " <<  m_imageHeader.dataLength << std::endl;
      std::cout << "ss = " <<  m_imageHeader.ss << std::endl;
      std::cout << "imageType = " <<  m_imageHeader.imageType << std::endl;
      std::cout << "frameWidth = " <<  m_imageHeader.frameWidth << std::endl;
      std::cout << "frameHeight = " <<  m_imageHeader.frameHeight << std::endl;
      std::cout << "transducerRadius = " <<  m_imageHeader.transducerRadius << std::endl;
      std::cout << "scanLinePitch = " <<  m_imageHeader.scanLinePitch << std::endl;
      std::cout << "scanLineNumber = " <<  m_imageHeader.scanLineNumber << std::endl;
      std::cout << "degPerFr = " <<  m_imageHeader.degPerFr << std::endl;
      std::cout << "framesPerVolume = " <<  m_imageHeader.framesPerVolume << std::endl;
    }

    m_grabbedImage.resize(m_imageHeader.frameWidth,m_imageHeader.frameHeight);

    m_bytesLeftToRead = m_imageHeader.dataLength;

    m_bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap,m_imageHeader.dataLength);

    if(m_bytesLeftToRead == 0 ) { // we've read all the frame in 1 packet.
      invertRowsCols();
      emit newFrameArrived(&m_outputImage);
    }
    if(m_verbose)
      std::cout << "Bytes left to read for whole frame = " << m_bytesLeftToRead << std::endl;

  }

  //we have a part of the image still not read (arrived with next tcp packet)
  else {
    if(m_verbose) {
      std::cout << "reading following part of the frame" << std::endl;
      std::cout << "local image size = " << m_grabbedImage.getSize() << std::endl;
    }
    m_bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap+(m_grabbedImage.getSize()-m_bytesLeftToRead),m_bytesLeftToRead);

    if(m_bytesLeftToRead==0) { // we've read the last part of the frame.
      invertRowsCols();
      emit newFrameArrived(&m_outputImage);
    }
  }
}

/**
* Method to invert rows and columns in the image.
*/
void usNetworkGrabberPreScan::invertRowsCols() {
  m_outputImage.resize(m_grabbedImage.getWidth(),m_grabbedImage.getHeight());

  for(unsigned int i=0; i<m_grabbedImage.getHeight(); i++)
    for (unsigned int j=0; j<m_grabbedImage.getWidth(); j++)
      m_outputImage(j,i,m_grabbedImage(i,j));
}

#endif
