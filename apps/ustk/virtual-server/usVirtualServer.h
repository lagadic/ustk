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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usVirtualServer.h
 * @brief Class to simulate a server sending frames from an ultrasound station.
 */
#ifndef US_VIRTUAL_SERVER_H
#define US_VIRTUAL_SERVER_H

#include <QApplication>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtCore/QDataStream>
#include <QtCore/QDateTime>
#include <QtCore/QString>
#include <QtCore/QFile>

#include <vector>
#include <cmath>
#include <ctime>
#include <iostream>
//#include <unistd.h>

//USTK inclues
#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/ustk_io/usMHDSequenceReader.h>

#include "usConsoleListener.h"

/**
 * @class usVirtualServer
 * @brief Class to simulate a server sending frames from an ultrasound station. Permits to replay a sequence of images sent through the network, by respeting the timestamps of each frame sent (to do real-time tests).
 */
class usVirtualServer : public QObject
{
  Q_OBJECT

public:

  // Following headers must be the same in the client grabber !

  //header sent by the client to init porta
  struct usInitHeaderIncomming{
    usInitHeaderIncomming() : headerId(1) {} // set header Id to 1 by default
    int headerId; //to differenciate usInitHeaderIncomming (=1) / usAcquisitionParameters (=2) / usRunControlHeaderSent(=3)

    //probe / slot selection
    int probeId; // available probes : 4DC7 (id = 15) , C5-2 (id = 10)
    int slotId; // 3 slots on ultrasonix station (0 : top, 2 : middle, 1 : botom)

    int imagingMode; // see ImagingModes.h
  };

  //header sent by the client to update porta config
  struct usUpdateHeaderIncomming{
    usUpdateHeaderIncomming() : headerId(2) {} // set header Id to 1 by default
    int headerId; //to differenciate usInitHeaderIncomming (=1) / usUpdateHeaderIncomming (=2)

    int transmitFrequency;
    int samplingFrequency;
    int imagingMode;
    bool postScanMode;
    int postScanHeight;
    int postScanWidth;
    int imageDepth;
    int sector;
    bool activateMotor;
    int motorPosition;
    int framesPerVolume;
    int stepsPerFrame;
  };

  //header sent to the client to confirm porta is initialized
  struct usInitHeaderConfirmation{
    usInitHeaderConfirmation() : headerId(1) {} // set header Id to 1 by default
    int headerId; //to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    int initOk; // 1 if init ok, 0 otherwise
    int probeId; // unique code for each probe (4DC7 = 15, C 50-62 = 10)
  };

  //
  struct usImageHeader{
    usImageHeader() : headerId(2) {} //set header Id to 2 by default
    int headerId; //to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    quint32 frameCount;
    quint64 timeStamp;

    double dataRate; // fps

    int dataLength; //image size in bytes
    int ss;

    int imageType;

    int frameWidth;
    int frameHeight;

    double pixelWidth; // width of a pixel of a post-scan frame (meters)
    double pixelHeight; // height of a pixel of a post-scan frame (meters)

    int transmitFrequency;
    int samplingFrequency;

    double transducerRadius;
    double scanLinePitch;
    unsigned int scanLineNumber;
    int imageDepth;

    double degPerFrame;
    int framesPerVolume;
    double motorRadius;
    int motorType;
  };

  explicit usVirtualServer(std::string sequencePath, QObject *parent = 0);
  ~usVirtualServer();

signals:
  void runAcquisitionSignal(bool run);

  void startSendingLoopSignal();

private slots:
  // Called automatically when a client attempts to connect
  void acceptTheConnection();

  // Called automatically when client has closed the connection
  void connectionAboutToClose();

  // Called when user decide to quit the pause (see usConsoleListener)
  void quitPause();

  // Called automatically when data sent by a client is fully available to the server
  void readIncomingData();

  void runAcquisition(bool run);

  void startSendingLoop();

private:

  void initServer(usInitHeaderIncomming header);

  void invertRowsColsOnPreScan();

  void setSequencePath(const std::string sequencePath);

  void sendingLoopSequenceXml();
  void sendingLoopSequenceMHD();

  bool updateServer(usUpdateHeaderIncomming header);

  void writeInitAcquisitionParameters(QDataStream & out, int imagingMode);

  // Variable(socket) to store listening tcpserver
  QTcpServer m_tcpServer;

  // Variable(socket) to store newly established connection with the client
  QTcpSocket * connectionSoc;
  usInitHeaderConfirmation confirmHeader;

  bool initWithoutUpdate;
  
#ifdef VISP_HAVE_XML2
  usSequenceReader<usImagePostScan2D <unsigned char> > m_sequenceReaderPostScan;
  usSequenceReader<usImagePreScan2D <unsigned char> > m_sequenceReaderPreScan;
#endif
  usMHDSequenceReader m_MHDSequenceReader;

  usImageHeader imageHeader;

  // 2D images
  usImagePostScan2D <unsigned char> m_postScanImage2d;
  usImagePreScan2D <unsigned char> m_preScanImage2d;
  usImageRF2D <short int> m_rfImage2d;

  // 3D images
  usImagePreScan3D <unsigned char> m_preScanImage3d;
  usImageRF3D <short int> m_rfImage3d;

  us::ImageType m_imageType;

  bool m_isMHDSequence; // mhd sequence if true, xml otherwise

  std::vector<uint64_t> m_timestamps; // vector of timestamps of the current volume (only for 3D case : RF and pre-scan volumes)
  uint64_t m_nextImageTimestamp;

  bool m_serverIsSendingImages;

  std::string m_sequencePath;

  //pause option
  bool m_usePause;
  bool m_pauseOn;
  unsigned int m_pauseImageNumber;
  uint64_t m_pauseDurationOffset;
  unsigned int m_pauseIndexOffset;
  // memory for pause on volumes (sending V, V+1, V, V+1 ...)
  usImagePreScan3D <unsigned char> m_preScanImage3dTemp;
  usImageRF3D <short int> m_rfImage3dTemp;
  std::vector<uint64_t> m_timestampsTemp;

  // For user inputs in console
  usConsoleListener m_consoleListener;
};

#endif // US_VIRTUAL_SERVER_H
