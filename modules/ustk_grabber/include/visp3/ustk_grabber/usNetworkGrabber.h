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
 * @brief Virtual class providing network tools to grab frames from ultrasonix station, using a tcp connection.
 */

#ifndef __usNetworkGrabber_h_
#define __usNetworkGrabber_h_

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5)

#include <QThread>
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
#elif defined( USTK_GRABBER_HAVE_QT5)
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QAbstractSocket>
#include <QtNetwork/QHostAddress>
#endif

/**
 * @class usNetworkGrabber
 * @brief Generic abstract class to manage tcp connection to grab ultrasound frames.
 * @ingroup module_ustk_grabber
 */
class VISP_EXPORT usNetworkGrabber : public QObject
{
  Q_OBJECT
public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  // Following headers must be the same in the server (ultrasound station) !

  //header sent by the client to init porta
  struct usInitHeaderSent{
    usInitHeaderSent() : headerId(1) {} // set header Id to 1 by default
    int headerId; //to differenciate usInitHeaderSent (=1) / usUpdateHeaderSent (=2)

    //probe / slot selection
    int probeId;
    int slotId;

    //frequencies
    int transmitFrequency;
    int samplingFrequency;

    //image type
    int imagingMode; // see ImagingModes.h
    bool postScanMode; //performs scan conversion on ultrasound station if true
    int postScanHeigh; // if post-scan mode, height of the frame (px)
    int postScanWidth; // if post-scan mode, width of the frame (px)

    int imageDepth; //in mm
    int sector; // in %

    //motor settings
    bool activateMotor; //to sweep the motor permanently

    // position of the motor in degrees : 0° = side of the fixation system for 4DC7 probe
    int motorPosition; // (used if activateMotor = false)

    // motor movement parameters
    int framesPerVolume; // (must be odd : always a central frame)
    int degreesPerFrame; // angle between two frames in degrees

  };

  //header sent by the client to update porta config
  struct usUpdateHeaderSent{
    usUpdateHeaderSent() : headerId(2) {} // set header Id to 2 by default
    int headerId; //to differenciate usInitHeaderSent (=1) / usUpdateHeaderSent (=2)

    int imagingMode; // see ImagingModes.h
    int imageHeight; //in px
    int frequency;
  };

  //header sent by the server to confirm porta is initialized
  struct usInitHeaderConfirmation{
    usInitHeaderConfirmation() : headerId(1) {} // set header Id to 1 by default
    int headerId; // Never change this value ! It is used to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    int initOk; // 1 if init ok, 0 otherwise
    int probeId; // unique code for each probe (4DC7 = 15, C 50-62 = 10)
  };

  // Header coming before every frame
  struct usImageHeader{
    usImageHeader() : headerId(2) {} //set header Id to 2 by default
    int headerId; // Never change this value ! It is used to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    quint32 frameCount; //from the beginning of last acquisition
    quint64 timeStamp; //msecs since epoch (on ultrasond machine)

    double dataRate; // in FPS

    int dataLength; //frame size in bytes, used to read on the network
    int ss;	// sample size in bits

    int imageType;	 	// type of data (0 = pre-scan, 1 = post-scan, 2 = rf)

    int frameWidth; // width of a frame (pixels for post-scan, scanlines for pre-scan or rf data)
    int frameHeight; // height of frame (pixels for post-scan, samples for pre-scan or rf data)

    int transmitFrequency; // in Hz
    int samplingFrequency; // in Hz

    //transducer settings
    double transducerRadius;
    double scanLinePitch;
    unsigned int scanLineNumber;

    // motor settings
    int degPerFr; // degree step between frames
    int framesPerVolume; //number of frames in a volume


  };
#endif //DOXYGEN_SHOULD_SKIP_THIS

  explicit usNetworkGrabber(QObject *parent = 0);
  ~usNetworkGrabber();

  void disconnect();

  void initAcquisition(const usNetworkGrabber::usInitHeaderSent &header);

  void setIPAddress(std::string s_ip){m_ip = s_ip;}

  void setVerbose(bool verbose) {m_verbose = verbose;}

public slots:
  /// Network
  void connected();
  void connectToServer();
  virtual void dataArrived() = 0;
  void disconnected();
  void handleError(QAbstractSocket::SocketError err);
  void setConnection(bool actionConnect);
  void useSimulator(bool t_state);
protected:

  bool m_verbose; //to print connection infos if true

  // Network
  QTcpSocket *m_tcpSocket;
  bool m_connect;
  std::string m_ip;

  //bytes to read until image end
  int m_bytesLeftToRead;

  //received headers
  usInitHeaderConfirmation m_confirmHeader;
  usImageHeader m_imageHeader;

};

#endif // QT4 || QT5
#endif // __usNetworkGrabber_h_
