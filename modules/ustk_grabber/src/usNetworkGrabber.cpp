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

#if defined(USTK_HAVE_QT5)
#include <QDataStream>
#endif

using namespace std;

usNetworkGrabber::usNetworkGrabber(QObject *parent) :
  QObject(parent)
{
  m_ip = "192.168.100.2";

  tcpSocket = new QTcpSocket(this);

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

void usNetworkGrabber::setConnection(bool a)
{
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
  // Notify an error. tcpSocket.errorString() automatically gets an error message (in english).
  throw(vpException(vpException::fatalError,tcpSocket->errorString().toStdString()));

  // Formally close the connection
  tcpSocket->close();
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
