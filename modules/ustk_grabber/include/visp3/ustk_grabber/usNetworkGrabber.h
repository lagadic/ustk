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

#include <visp3/ustk_core/usConfig.h>
#include <visp3/ustk_core/us.h>
#include <visp3/ustk_grabber/usAcquisitionParameters.h>

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <cassert>
#include <iostream>
#include <cstring>

//Qt Network
#include <QtCore/QThread>
#include <QtCore/QObject>
#include <QtCore/QThread>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>

/**
 * @class usNetworkGrabber
 * @brief Generic abstract class to manage tcp connection to grab ultrasound frames (on port 8080).
 * @ingroup module_ustk_grabber


 This class contains all the network process to manage the remote control the acquisition (the code for server side is on lagadic gitlab).

 The following figure details the network communication process and summarizes the steps to follow to acquire ultrasound images :
 \image html img-usNetworkGrabber.png
 */
class VISP_EXPORT usNetworkGrabber : public QObject
{
  Q_OBJECT
public:

  typedef enum {
    OUTPUT_FRAME_POSITION_IN_VEC = 0,
    MOST_RECENT_FRAME_POSITION_IN_VEC = 1,
    CURRENT_FILLED_FRAME_POSITION_IN_VEC = 2
  }usDataPositionInBuffer;

  // Following headers must be the same in the server (ultrasound station) !

  /**
   * header sent by the client to init porta on server side.
   */
  struct usInitHeaderSent{
    usInitHeaderSent() : headerId(1) {} // set header Id to 1 by default
    int headerId; /**< header id to differenciate usInitHeaderSent (=1) / usAcquisitionParameters (=2) / usRunControlHeaderSent(=3) */

  /**
   * @name probe / slot selection
   */
  /*@{*/
    int probeId; /**< available probes : 4DC7 (id = 15) , C5-2 (id = 10) */
    int slotId; /**<  3 slots on ultrasonix station (0 : top, 2 : middle, 1 : botom) */
  /*@}*/
    int imagingMode; /**< see ImagingModes.h in porta SDK */
  };

  /**
   * header sent by the client to run/stop the acquisition on server side.
   */
  struct usRunControlHeaderSent{
    usRunControlHeaderSent() : headerId(3), run(false) {} // set header Id to 1 by default
    int headerId; /**< header id to differenciate usInitHeaderSent (=1) / usAcquisitionParameters (=2) / usRunControlHeaderSent(=3) */

    bool run; /**< boolean to run (true) or stop (false) the aquisition */
  };

  /**
   * header sent by the server to confirm porta is initialized
   */
  struct usInitHeaderConfirmation{
    usInitHeaderConfirmation() : headerId(1), initOk(0), probeId(0) {} // set header Id to 1 by default
    int headerId; /**<  Never change this value ! It is used to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2) */

    int initOk; /**<  1 if init ok, 0 otherwise */
    int probeId; /**<  unique code for each probe (4DC7 = 15, C 50-62 = 10) */
  };

  explicit usNetworkGrabber(QObject *parent = 0);
  ~usNetworkGrabber();

  void disconnect();

  bool initAcquisition(const usNetworkGrabber::usInitHeaderSent &header);

  bool getMotorActivation();
  usAcquisitionParameters::usMotorStep getStepsPerFrame();
  int getFramesPerVolume();
  int getImageDepth();
  int getImagingMode() ;
  int getMotorPosition();
  int getPostScanHeigh();
  bool getPostScanMode();
  int getPostScanWidth();
  int getSamplingFrequency();
  int getSector();
  int getTransmitFrequency();

  void readAcquisitionParameters(QDataStream &stream);

  void runAcquisition();

  bool sendAcquisitionParameters();

  void setIPAddress(const std::string &s_ip){m_ip = s_ip;}

  void setMotorActivation(bool activateMotor);
  void setStepsPerFrame(usAcquisitionParameters::usMotorStep stepsPerFrame);
  void setFramesPerVolume(int framesPerVolume);
  void setImageDepth(int imageDepth);
  void setImagingMode(int imagingMode) ;
  void setMotorPosition(int motorPosition);
  void setPostScanHeigh(int postScanHeigh);
  void setPostScanMode(bool postScanMode);
  void setPostScanWidth(int postScanWidth);
  void setSamplingFrequency(int samplingFrequency);
  void setSector(int sector);
  void setTransmitFrequency(int transmitFrequency);

  void setVerbose(bool verbose) {m_verbose = verbose;}

  void stopAcquisition();

signals:
  void serverUpdateEnded(bool success);
  void endBlockingLoop();
  void runAcquisitionSignal(bool run);
  void sendAcquisitionParametersSignal();


public slots:
  /// Network
  void connected();
  void processConnectionToServer();
  virtual void dataArrived() = 0;
  void disconnected();
  void handleError(QAbstractSocket::SocketError err);
  void connectToServer();
  void disconnectFromServer();
  void setServerIp(std::string & ip);

protected slots:
  void serverUpdated(bool sucess);
  void sendRunSignal(bool run);
  void sendAcquisitionParametersSlot();

protected:

  bool m_verbose; //to print connection infos if true

  // Network
  QTcpSocket *m_tcpSocket;
  bool m_connect;
  std::string m_ip;

  //bytes to read until image end
  int m_bytesLeftToRead;

  //acquisition parameters
  usAcquisitionParameters m_acquisitionParamters;

  //To know if the update was sucessfull on server side
  bool m_updateParametersSucess;

  //received headers
  usInitHeaderConfirmation m_confirmHeader;
  us::usImageHeader m_imageHeader;

  // grabber state
  bool m_isInit;
  bool m_isRunning;
};

#endif // QT4 || QT5
#endif // __usNetworkGrabber_h_
