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

#include <visp3/ustk_grabber/usNetworkGrabber.h>

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

#include <iostream>
#include <fstream>
#include <visp3/io/vpImageIo.h>
using namespace std;

usNetworkGrabber::usNetworkGrabber(QObject *parent) :
  QObject(parent)
{
  m_ip = "192.168.100.2";
  initialize();

  m_grabbedImage.init(128,448);

  bytesLeftToRead = 0;
}

usNetworkGrabber::~usNetworkGrabber()
{
  if(tcpSocket->isOpen())
    tcpSocket->close();
}

void usNetworkGrabber::useSimulator(bool t_state)
{
  m_ip = (t_state)?"127.0.0.1":"192.168.100.2";
}


void usNetworkGrabber::initialize()
{
  tcpSocket = new QTcpSocket(this);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////    Network events     /////////////////////////////////////////////////
///////////////////////////////////  version US Machine   /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void usNetworkGrabber::setConnection(bool a)
{
  currentIdx = 0;
  oldIdx = -1;
  m_start = false;

  m_connect = a;
  if(a)
    this->ActionConnect();
  else
    tcpSocket->disconnect();
}

void usNetworkGrabber::ActionConnect()
{
  if(m_connect)
  {
    std::cout << "ip listening : " << m_ip.c_str() << std::endl;
    QHostAddress addr(m_ip.c_str());
    //tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    tcpSocket->connectToHost(addr, 8080);

    connect(tcpSocket,SIGNAL(connected()),this,SLOT(connected()));
    connect(tcpSocket,SIGNAL(disconnected()),this,SLOT(disconnected()));
    connect(tcpSocket ,SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(handleError(QAbstractSocket::SocketError)));
    connect(tcpSocket ,SIGNAL(readyRead()),this, SLOT(dataArrived()));

    if (tcpSocket->isOpen ())
      std::cout << "socket is open." << std::endl;
    else
      std::cout << "socket not is open." << std::endl;

    if ( tcpSocket->isReadable())
      std::cout << "socket is readable." << std::endl;
  }
  else
    tcpSocket->close();
}

void usNetworkGrabber::connected()
{
  std::cout << "connected to server" << std::endl;
  std::cout << "local port : " << tcpSocket->localPort() << std::endl;
  std::cout << "local addr : " << tcpSocket->localAddress().toString().toStdString() << std::endl;
  std::cout << "peer port : " << tcpSocket->peerPort() << std::endl;
  std::cout << "peer addr : " << tcpSocket->peerAddress().toString().toStdString() << std::endl;
}

void usNetworkGrabber::disconnected()
{
  std::cout <<"Disconnected .... \n";
  tcpSocket->close();
}

// This function is called if there is an error (or disruption) in the connection
void usNetworkGrabber::handleError(QAbstractSocket::SocketError err)
{
  Q_UNUSED(err);
  // Pop-up to notify an error. tcpSocket.errorString() automatically gets an error message (in english).
  QMessageBox::critical(NULL,"Error", tcpSocket->errorString());

  // Formally close the connection
  tcpSocket->close();
}


// This function is called when the data is fully arrived from the server to the client
void usNetworkGrabber::dataArrived()
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
  if(bytesLeftToRead == 0 ) { // do not read a header if last frame was not complete
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
      QString str = QString("test") + QString::number(imageHeader.frameCount) + QString(".png");
      vpImageIo::write(m_grabbedImage,str.toStdString());
    }

    std::cout << "left to read= " << bytesLeftToRead << std::endl;

  }
  //we have a part of the image still not read (arrived with next tcp packet)
  else {
    bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap+(m_grabbedImage.getSize()-bytesLeftToRead),bytesLeftToRead);

    if(bytesLeftToRead==0) {
      QString str = QString("test") + QString::number(imageHeader.frameCount) + QString(".png");
      vpImageIo::write(m_grabbedImage,str.toStdString());
       }
    /*tcpSocket->close();
    throw(vpException(vpException::fatalError, "unknown data received, closing connection."));*/
  }
}

void usNetworkGrabber::initAcquisition(usNetworkGrabber::usInitHeaderSent header) {
  std::cout << "init acquisition" << std::endl;

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  #if (defined(USTK_HAVE_QT5))
    out.setVersion(QDataStream::Qt_5_0);
  #elif (defined(USTK_HAVE_QT4))
    out.setVersion(QDataStream::Qt_4_8);
  #else
    throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
  #endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)
  out << header.imagingMode;
  out << header.imageHeight;
  out << header.frequency;
  tcpSocket->write(block);
}

void usNetworkGrabber::stopAcquisition() {
  tcpSocket->disconnect();
}
#endif
