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

#ifdef USTK_HAVE_QT4 || USTK_HAVE_QT5

usNetworkGrabber::usNetworkGrabber(QWidget *parent) :
    QObject(parent)
{
    m_ip = "192.168.100.2";
	initialize();
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

    connect(tcpSocket ,SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(handleError(QAbstractSocket::SocketError)));
    connect(tcpSocket ,SIGNAL(readyRead()),this, SLOT(dataArrived()));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////    Network events     /////////////////////////////////////////////////
///////////////////////////////////  version US Machine   /////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void usNetworkGrabber::setConnection(bool a)
{
    currentIdx = 0;
    oldIdx = -1;
    mSizeData = 0;
    m_start = false;

    m_connect = a;
    this->ActionConnect();
}

void usNetworkGrabber::ActionConnect()
{
    if(m_connect)
    {
        QHostAddress addr(m_ip.c_str());
        tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        tcpSocket->connectToHost(addr, 8080);
        this->sendEnabled();
    }
    else
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
    QDataStream ds(tcpSocket);
    ds.setVersion(QDataStream::Qt_5_2);

    // before each packet we are expected the size of the data to be received
   if (mSizeData == 0)
   {
     if (tcpSocket->bytesAvailable() < (quint64)sizeof(quint64))
     {
       return; // not received enough data to fill the data size
     }
     ds >> mSizeData;
   }

   // Buffer data until the whole init packet is received
   if (tcpSocket->bytesAvailable() < mSizeData)
     return;

   // Getting buffer data
   // 0. Index
   int t_idx;
   ds >> t_idx;
   if(t_idx == 0)
       m_start = true;
   currentIdx = t_idx;
   // 1. Get the header
   char *t_hdr;
   uint len;
   ds.readBytes(t_hdr, len);
   int *hdrx = (int*)malloc(len);
   memcpy(hdrx, t_hdr, len);
   // 2. Get the data
   char *t_data;
   uint lenD;
   ds.readBytes(t_data, lenD);
   short *dData = (short*)malloc(lenD);
   memcpy(dData, t_data, lenD);

   // Emiting data
   if((currentIdx != oldIdx) && m_start)
   {
        emit sendBuffer(dData, currentIdx);
   }
   // 3. No more data
   mSizeData=0;
   // 4. New data requested
   oldIdx = currentIdx;

   free(hdrx);
   free(dData);
   free(t_hdr);
   free(t_data);

   sendEnabled();

   tcpSocket->flush();
}


void usNetworkGrabber::sendEnabled()
{
    QString str = QString::number(1);
    tcpSocket->write(str.toUtf8());
}

#endif
