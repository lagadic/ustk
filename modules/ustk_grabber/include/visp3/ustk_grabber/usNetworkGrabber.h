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
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QHostAddress>


class usNetworkGrabber : public QObject
{
    Q_OBJECT
public:
    explicit usNetworkGrabber(QWidget *parent = 0);
    ~usNetworkGrabber();
	void SetIPAddress(std::string s_ip){m_ip = s_ip;}
signals:
    void sendBuffer(short *tbuff, int tdx);
public slots:
    /// Network
    void initialize(void);
    void setConnection(bool a);
    void ActionConnect();
    void handleError(QAbstractSocket::SocketError err);
    void dataArrived();
    void sendEnabled();
	void useSimulator(bool t_state);
private:
    bool f_loaded;
    int currentIdx;
    int oldIdx;
    bool m_start;
    double wr, hr;
    // Network
    QTcpSocket *tcpSocket;
    qint64 bytesExpected;
    quint64 mSizeData;
    bool m_connect;
	std::string m_ip;
};

#endif // USTK_HAVE_QT4
#endif // __usNetworkGrabber_h_
