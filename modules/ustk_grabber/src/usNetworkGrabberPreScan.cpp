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

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

#include<visp3/io/vpImageIo.h>

#if defined(USTK_HAVE_QT5)
#include <QDataStream>
#endif

usNetworkGrabberPreScan::usNetworkGrabberPreScan(usNetworkGrabber *parent) :
  usNetworkGrabber(parent)
{
  m_grabbedImage.init(0,0);
  connect(tcpSocket ,SIGNAL(readyRead()),this, SLOT(dataArrived()));
}

usNetworkGrabberPreScan::~usNetworkGrabberPreScan()
{

}

// This function is called when the data is fully arrived from the server to the client
void usNetworkGrabberPreScan::dataArrived()
{
  std::cout << "dataArrived" << std::endl;

  ////////////////// HEADER READING //////////////////
  QDataStream in;
  in.setDevice(tcpSocket);
#if (defined(USTK_HAVE_QT5))
  in.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  in.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  int headerType;
  if(bytesLeftToRead == 0 ) { // do not try to read a header if last frame was not complete
    in >> headerType;
    std::cout << "header received, type = " << headerType << std::endl;
  }
  else {
    headerType = 0;
  }
  //init confirm header received
  if(headerType == confirmHeader.headerId) {
    //read whole header
    in >> confirmHeader.initOk;
    in >> confirmHeader.probeId;

    if(confirmHeader.initOk == 0) {
      tcpSocket->close();
      throw(vpException(vpException::fatalError, "porta initialisation error, closing connection."));
    }

    std::cout << "porta init sucess, detected probe id = " << confirmHeader.probeId << std::endl;

  }

  //image header received
  else if(headerType == imageHeader.headerId) {
    //read whole header
    in >> imageHeader.frameCount;
    in >> imageHeader.heightPx;
    in >> imageHeader.heightMeters;
    in >> imageHeader.widthPx;
    in >> imageHeader.widthtMeters;
    in >> imageHeader.timeStamp;
    in >> imageHeader.dataLength;

    std::cout << "frameCount = " <<  imageHeader.frameCount << std::endl;
    std::cout << "heightPx = " <<  imageHeader.heightPx << std::endl;
    std::cout << "heightMeters = " <<  imageHeader.heightMeters << std::endl;
    std::cout << "widthPx = " <<  imageHeader.widthPx << std::endl;
    std::cout << "widthtMeters = " <<  imageHeader.widthtMeters << std::endl;
    std::cout << "timeStamp = " <<  imageHeader.timeStamp << std::endl;
    std::cout << "dataLength = " <<  imageHeader.dataLength << std::endl;

    m_grabbedImage.resize(imageHeader.widthPx,imageHeader.heightPx);

    bytesLeftToRead = imageHeader.dataLength;

    bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap,imageHeader.dataLength);

    if(bytesLeftToRead == 0 ) { // we've read all the frame
      //QString str = QString("test") + QString::number(imageHeader.frameCount) + QString(".png");
      //vpImageIo::write(m_grabbedImage,str.toStdString());
      emit newFrameArrived(&m_grabbedImage);
    }

    std::cout << "left to read= " << bytesLeftToRead << std::endl;

  }
  //we have a part of the image still not read (arrived with next tcp packet)
  else {
    bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap+(m_grabbedImage.getSize()-bytesLeftToRead),bytesLeftToRead);

    if(bytesLeftToRead==0) {
      emit newFrameArrived(&m_grabbedImage);
      //QString str = QString("test") + QString::number(imageHeader.frameCount) + QString(".png");
      //vpImageIo::write(m_grabbedImage,str.toStdString());
    }
  }
}

#endif
