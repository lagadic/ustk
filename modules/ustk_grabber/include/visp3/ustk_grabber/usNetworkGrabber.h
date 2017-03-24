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

/**
 * @file usNetworkGrabber.h
 * @brief Grabber used to grab frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkGrabber_h_
#define __usNetworkGrabber_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

#include <QThread>
#include <QMessageBox>
#include <cassert>
#include <QObject>
#include <QScopedPointer>
#include <iostream>
#include <cstring>

//Qt Network
#if defined(USTK_HAVE_QT4)
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QHostAddress>
#elif defined( USTK_HAVE_QT5)
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QAbstractSocket>
#include <QtNetwork/QHostAddress>
#endif

#include <visp3/ustk_core/usImagePreScan2D.h>

class VISP_EXPORT usNetworkGrabber : public QObject
{
    Q_OBJECT
public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  // Following headers must be the same in the client grabber !

    //header sent by the client to init porta
    struct usInitHeaderSent{
      int imagingMode; // see ImagingModes.h
      int imageHeight; //in px
      int frequency;
    };

    //header sent by the server to confirm porta is initialized
    struct usInitHeaderConfirmation{
      usInitHeaderConfirmation() : headerId(1) {} // set header Id to 1 by default
      int headerId; //to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

      int initOk; // 1 if init ok, 0 otherwise
      int probeId; // unique code for each probe (4DC7 = 15, C 50-62 = ?)
    };

    // Header coming before every frame
    struct usImageHeader{
      usImageHeader() : headerId(2) {} //set header Id to 2 by default
      int headerId; //to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

      quint32 frameCount;
      int heightPx; //px
      float heightMeters; //meters
      int widthPx; //px
      float widthtMeters; //meters

      double timeStamp;

      int dataLength; //in bytes

    };
#endif //DOXYGEN_SHOULD_SKIP_THIS

    explicit usNetworkGrabber(QObject *parent = 0);
    ~usNetworkGrabber();
  void SetIPAddress(std::string s_ip){m_ip = s_ip;}

  void initAcquisition(usNetworkGrabber::usInitHeaderSent header);
  void stopAcquisition();

public slots:
    /// Network
    void initialize(void);
    void setConnection(bool a);
    void ActionConnect();
    void handleError(QAbstractSocket::SocketError err);
    void dataArrived();
  void useSimulator(bool t_state);
  void connected();
  void disconnected();
private:
    bool f_loaded;
    int currentIdx;
    int oldIdx;
    bool m_start;
    // Network
    QTcpSocket *tcpSocket;
    bool m_connect;
	std::string m_ip;

  //bytes to read again until image end
  int bytesLeftToRead;

  //received headers
  usInitHeaderConfirmation confirmHeader;
  usImageHeader imageHeader;


  usImagePreScan2D<unsigned char> m_grabbedImage;

};

#endif // USTK_HAVE_QT4
#endif // __usNetworkGrabber_h_
