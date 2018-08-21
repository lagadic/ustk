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

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <fstream>
#include <iostream>
#include <visp3/io/vpImageIo.h>

#include <QtCore/QDataStream>
#include <QtCore/QEventLoop>

using namespace std;

/**
* Constructor.
* @param parent Parent QObject.
*/
usNetworkGrabber::usNetworkGrabber(QObject *parent) : QObject(parent)
{
  m_ip = "192.168.100.2";

  m_tcpSocket = new QTcpSocket(this);

  m_bytesLeftToRead = 0;

  m_verbose = false;

  m_connect = false;

  m_acquisitionParameters = usAcquisitionParameters();
  m_confirmHeader = usInitHeaderConfirmation();
  m_imageHeader = us::usImageHeader();

  m_isInit = false;
  m_isRunning = false;
  
  m_thread = NULL;

  QObject::connect(this, SIGNAL(serverUpdateEnded(bool)), this, SLOT(serverUpdated(bool)));
  QObject::connect(this, SIGNAL(runAcquisitionSignal(bool)), this, SLOT(sendRunSignal(bool)));
  QObject::connect(this, SIGNAL(sendAcquisitionParametersSignal()), this, SLOT(sendAcquisitionParametersSlot()));
}

/**
* Destructor.
*/
usNetworkGrabber::~usNetworkGrabber()
{
  this->stopAcquisition();
  if(m_thread) {
    m_thread->quit();
    m_thread->wait();
    delete m_thread;
    m_thread = NULL;
  }
  if (m_tcpSocket->isOpen())
    m_tcpSocket->close();
}

/**
* Method used to connect to the server.
*/
void usNetworkGrabber::connectToServer()
{
  m_connect = true;
  this->processConnectionToServer();
}

/**
* Method used to connect to the server.
*/
void usNetworkGrabber::connectToServer(QHostAddress address)
{
  setIPAddress(address.toString().toStdString());
  m_connect = true;
  this->processConnectionToServer();
}

/**
* Method used to disconnect from the server.
*/
void usNetworkGrabber::disconnectFromServer()
{
  m_connect = false;

  emit(endConnection());
  // wait disconnection is effective
  QEventLoop loop;
  loop.connect(m_tcpSocket, SIGNAL(disconnected()), SLOT(quit()));
  loop.exec();
}

/**
* Method used to do all the process of the server connection.
*/
void usNetworkGrabber::processConnectionToServer()
{
  if (m_connect) {
    if (m_verbose)
      std::cout << "ip listening : " << m_ip.c_str() << std::endl;
    QHostAddress addr(m_ip.c_str());
    m_tcpSocket->connectToHost(addr, 8080);

    connect(m_tcpSocket, SIGNAL(connected()), this, SLOT(connected()));
    connect(this, SIGNAL(endConnection()), this, SLOT(disconnected()));
    connect(m_tcpSocket, SIGNAL(disconnected()), this, SLOT(disconnected()));
    connect(m_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this,
            SLOT(handleError(QAbstractSocket::SocketError)));

    if (m_tcpSocket->isOpen()) {
      if (m_verbose) {
        std::cout << "socket is open." << std::endl;
      }
    } else {
      if (m_verbose) {
        std::cout << "socket not is open." << std::endl;
      }
    }

    if (m_tcpSocket->isReadable()) {
      if (m_verbose) {
        std::cout << "socket is readable." << std::endl;
      }
    }
  } else
    m_tcpSocket->close();
}

/**
* Slot called when the grabber is connected to the server. Prints connection informations.
*/
void usNetworkGrabber::connected()
{
  if (m_verbose) {
    std::cout << "connected to server" << std::endl;
    std::cout << "local port : " << m_tcpSocket->localPort() << std::endl;
    std::cout << "local addr : " << m_tcpSocket->localAddress().toString().toStdString() << std::endl;
    std::cout << "peer port : " << m_tcpSocket->peerPort() << std::endl;
    std::cout << "peer addr : " << m_tcpSocket->peerAddress().toString().toStdString() << std::endl;
  }
}

/**
* Slot called when the grabber is disconnected from the server. Prints information, and closes socket.
*/
void usNetworkGrabber::disconnected()
{
  if (m_verbose)
    std::cout << "Disconnected .... \n";
  m_tcpSocket->close();
}

/**
* Slot called if there is an error (or disruption) in the connection. Throws an exception and prints the error.
*/
void usNetworkGrabber::handleError(QAbstractSocket::SocketError err)
{
  Q_UNUSED(err);
  // Notify an error. tcpSocket.errorString() automatically gets an error message (in english).
  throw(vpException(vpException::fatalError, m_tcpSocket->errorString().toStdString()));

  // Formally close the connection
  m_tcpSocket->close();
}

/**
* Method called to init the ultrasonix station, by passing acquisition parameters.
* It is a blocking method : we wait the answer of the server to know if init was sucessfull.
* @param header Contains acquisition parameters to set up for the acquisition.
* @return Boolean to say if init was sucessfull (true) or not (false).
* @see usNetworkGrabber::usInitHeaderSent
*/
bool usNetworkGrabber::initAcquisition(const usNetworkGrabber::usInitHeaderSent &header)
{

  if (m_isRunning)
    stopAcquisition();

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif defined(USTK_HAVE_VTK_QT4)
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError, "your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)

  out << header.headerId;
  out << header.probeId;
  out << header.slotId;
  out << header.imagingMode;
  m_tcpSocket->write(block);

  if (m_verbose)
    std::cout << "INIT SENT, waiting server confirmation..." << std::endl;
  // loop to wait update on server
  QEventLoop *loop = new QEventLoop;
  QObject::connect(this, SIGNAL(endBlockingLoop()), loop, SLOT(quit()));
  loop->exec();

  m_isInit = m_updateParametersSucess;

  if (m_verbose)
    std::cout << "server confirmation : " << (int)m_updateParametersSucess << std::endl;

  return m_updateParametersSucess;
}

/**
* Slot to call to init the ultrasonix station, by passing acquisition parameters.
* It is a blocking method : we wait the answer of the server to know if init was sucessfull.
* @return Boolean to say if init was sucessfull (true) or not (false).
* @see usNetworkGrabber::usInitHeaderSent
*/
void usNetworkGrabber::initAcquisitionSlot(usInitHeaderSent header)
{

  if (m_isRunning)
    stopAcquisition();

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif defined(USTK_HAVE_VTK_QT4)
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError, "your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)

  out << header.headerId;
  out << header.probeId;
  out << header.slotId;
  out << header.imagingMode;
  m_tcpSocket->write(block);

  if (m_verbose)
    std::cout << "INIT SENT, waiting server confirmation..." << std::endl;
  // loop to wait update on server
  QEventLoop *loop = new QEventLoop;
  QObject::connect(this, SIGNAL(endBlockingLoop()), loop, SLOT(quit()));
  loop->exec();

  m_isInit = m_updateParametersSucess;

  if (m_verbose)
    std::cout << "server confirmation : " << (int)m_updateParametersSucess << std::endl;

  emit(acquisitionInitialized(m_isInit));
}

/**
* Method to close the connection.
*/
void usNetworkGrabber::disconnect() { m_tcpSocket->disconnect(); }

/**
* Method to send to the server the new acquisition parameters.
*/
bool usNetworkGrabber::sendAcquisitionParameters()
{

  if (m_isRunning)
    stopAcquisition();

  emit sendAcquisitionParametersSignal();

  return m_updateParametersSucess;
}

/**
  * Slot to send to the server the new acquisition parameters.
  */
void usNetworkGrabber::sendAcquisitionParametersSlot()
{

  QByteArray block;
  QDataStream stream(&block, QIODevice::WriteOnly);
#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT5))
  stream.setVersion(QDataStream::Qt_5_0);
#elif defined(USTK_HAVE_VTK_QT4)
  stream.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError, "your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)

  stream << 2; // update id = 2
  stream << m_acquisitionParameters.getTransmitFrequency();
  stream << m_acquisitionParameters.getSamplingFrequency();
  stream << m_acquisitionParameters.getImagingMode();
  stream << m_acquisitionParameters.getPostScanMode();
  stream << m_acquisitionParameters.getPostScanHeigh();
  stream << m_acquisitionParameters.getPostScanWidth();
  stream << m_acquisitionParameters.getImageDepth();
  stream << m_acquisitionParameters.getSector();
  stream << m_acquisitionParameters.getActivateMotor();
  stream << m_acquisitionParameters.getMotorPosition();
  stream << m_acquisitionParameters.getFramesPerVolume();
  stream << m_acquisitionParameters.getSepsPerFrame();

  if (m_verbose) {
    std::cout << "UPDATE SENT : " << std::endl;
    std::cout << "TransmitFrequency = " << m_acquisitionParameters.getTransmitFrequency() << std::endl;
    std::cout << "SamplingFrequency = " << m_acquisitionParameters.getSamplingFrequency() << std::endl;
    std::cout << "ImagingMode = " << m_acquisitionParameters.getImagingMode() << std::endl;
    std::cout << "PostScanMode = " << m_acquisitionParameters.getPostScanMode() << std::endl;
    std::cout << "PostScanHeigh = " << m_acquisitionParameters.getPostScanHeigh() << std::endl;
    std::cout << "PostScanWidth = " << m_acquisitionParameters.getPostScanWidth() << std::endl;
    std::cout << "ImageDepth = " << m_acquisitionParameters.getImageDepth() << std::endl;
    std::cout << "Sector = " << m_acquisitionParameters.getSector() << std::endl;
    std::cout << "ActivateMotor = " << m_acquisitionParameters.getActivateMotor() << std::endl;
    std::cout << "MotorPosition = " << m_acquisitionParameters.getMotorPosition() << std::endl;
    std::cout << "FramesPerVolume = " << m_acquisitionParameters.getFramesPerVolume() << std::endl;
    std::cout << "anglePerFrame = " << m_acquisitionParameters.getSepsPerFrame() << std::endl;
  }
  m_tcpSocket->write(block);

  if (m_verbose)
    std::cout << "waiting server confirmation..." << std::endl;

  // loop to wait update on server side
  QEventLoop *loop = new QEventLoop;
  QObject::connect(this, SIGNAL(endBlockingLoop()), loop, SLOT(quit()));
  loop->exec();

  if (m_verbose)
    std::cout << "Update server confirmation : " << (int)m_updateParametersSucess << std::endl;
}

/**
* Slot called when we get the answer from the server to our reques to update aquisition parameters.
*/
void usNetworkGrabber::serverUpdated(bool sucess)
{
  m_updateParametersSucess = sucess;
  emit(endBlockingLoop());
}

/**
* Method to read all parameters comming from the server (in answer to an update). It fills the usNetworkGrabber
* attribute.
*/
void usNetworkGrabber::readAcquisitionParameters(QDataStream &stream)
{

  int transmitFrequency;
  int samplingFrequency;
  int imagingMode;
  bool postScanMode;
  int postScanHeigh;
  int postScanWidth;
  int imageDepth;
  int sector;
  bool activateMotor;
  int motorPosition;
  int framesPerVolume;
  int anglePerFrame;
  int transmitFrequencyMin;
  int samplingFrequencyMin;
  int imagingModeMin;
  int imageDepthMin;
  int sectorMin;
  int motorPositionMin;
  int framesPerVolumeMin;
  int anglePerFrameMin;
  int transmitFrequencyMax;
  int samplingFrequencyMax;
  int imagingModeMax;
  int imageDepthMax;
  int sectorMax;
  int motorPositionMax;
  int framesPerVolumeMax;
  int anglePerFrameMax;

  stream >> transmitFrequency;
  stream >> samplingFrequency;
  stream >> imagingMode;
  stream >> postScanMode;
  stream >> postScanHeigh;
  stream >> postScanWidth;
  stream >> imageDepth;
  stream >> sector;
  stream >> activateMotor;
  stream >> motorPosition;
  stream >> framesPerVolume;
  stream >> anglePerFrame;
  stream >> transmitFrequencyMin;
  stream >> samplingFrequencyMin;
  stream >> imagingModeMin;
  stream >> imageDepthMin;
  stream >> sectorMin;
  stream >> motorPositionMin;
  stream >> framesPerVolumeMin;
  stream >> anglePerFrameMin;
  stream >> transmitFrequencyMax;
  stream >> samplingFrequencyMax;
  stream >> imagingModeMax;
  stream >> imageDepthMax;
  stream >> sectorMax;
  stream >> motorPositionMax;
  stream >> framesPerVolumeMax;
  stream >> anglePerFrameMax;

  if (m_verbose) {
    std::cout << "transmitFrequency = " << transmitFrequency << std::endl;
    std::cout << "samplingFrequency = " << samplingFrequency << std::endl;
    std::cout << "imagingMode = " << imagingMode << std::endl;
    std::cout << "postScanMode = " << postScanMode << std::endl;
    std::cout << "postScanHeigh = " << postScanHeigh << std::endl;
    std::cout << "postScanWidth = " << postScanWidth << std::endl;
    std::cout << "imageDepth = " << imageDepth << std::endl;
    std::cout << "sector = " << sector << std::endl;
    std::cout << "activateMotor = " << activateMotor << std::endl;
    std::cout << "motorPosition = " << motorPosition << std::endl;
    std::cout << "framesPerVolume = " << framesPerVolume << std::endl;
    std::cout << "anglePerFrame = " << anglePerFrame << std::endl;
    std::cout << "transmitFrequencyMin = " << transmitFrequencyMin << std::endl;
    std::cout << "samplingFrequencyMin = " << samplingFrequencyMin << std::endl;
    std::cout << "imagingModeMin = " << imagingModeMin << std::endl;
    std::cout << "imageDepthMin = " << imageDepthMin << std::endl;
    std::cout << "sectorMin = " << sectorMin << std::endl;
    std::cout << "motorPositionMin = " << motorPositionMin << std::endl;
    std::cout << "framesPerVolumeMin = " << framesPerVolumeMin << std::endl;
    std::cout << "anglePerFrameMin = " << anglePerFrameMin << std::endl;
    std::cout << "transmitFrequencyMax = " << transmitFrequencyMax << std::endl;
    std::cout << "samplingFrequencyMax = " << samplingFrequencyMax << std::endl;
    std::cout << "imagingModeMax = " << imagingModeMax << std::endl;
    std::cout << "imageDepthMax = " << imageDepthMax << std::endl;
    std::cout << "sectorMax = " << sectorMax << std::endl;
    std::cout << "motorPositionMax = " << motorPositionMax << std::endl;
    std::cout << "framesPerVolumeMax = " << framesPerVolumeMax << std::endl;
    std::cout << "anglePerFrameMax = " << anglePerFrameMax << std::endl;
  }

  m_acquisitionParameters.setTransmitFrequency(transmitFrequency);
  m_acquisitionParameters.setSamplingFrequency(samplingFrequency);
  m_acquisitionParameters.setImagingMode(imagingMode);
  m_acquisitionParameters.setPostScanMode(postScanMode);
  m_acquisitionParameters.setPostScanHeigh(postScanHeigh);
  m_acquisitionParameters.setPostScanWidth(postScanWidth);
  m_acquisitionParameters.setImageDepth(imageDepth);
  m_acquisitionParameters.setSector(sector);
  m_acquisitionParameters.setActivateMotor(activateMotor);
  m_acquisitionParameters.setMotorPosition(motorPosition);
  m_acquisitionParameters.setFramesPerVolume(framesPerVolume);
  if (anglePerFrame == 0)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_STATIC_MOTOR);
  else if (anglePerFrame == 2)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_1);
  else if (anglePerFrame == 4)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_2);
  else if (anglePerFrame == 8)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  else if (anglePerFrame == 16)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_4);
  else if (anglePerFrame == 32)
    m_acquisitionParameters.setSepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_5);

  m_acquisitionParameters.setTransmitFrequencyMin(transmitFrequencyMin);
  m_acquisitionParameters.setSamplingFrequencyMin(samplingFrequencyMin);
  m_acquisitionParameters.setImagingModeMin(imagingModeMin);
  m_acquisitionParameters.setImageDepthMin(imageDepthMin);
  m_acquisitionParameters.setSectorMin(sectorMin);
  m_acquisitionParameters.setMotorPositionMin(motorPositionMin);
  if (anglePerFrame == 0)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_STATIC_MOTOR);
  else if (anglePerFrame == 2)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_ANGLE_PITCH_1);
  else if (anglePerFrame == 4)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_ANGLE_PITCH_2);
  else if (anglePerFrame == 8)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_ANGLE_PITCH_3);
  else if (anglePerFrame == 16)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_ANGLE_PITCH_4);
  else if (anglePerFrame == 32)
    m_acquisitionParameters.setSepsPerFrameMin(usAcquisitionParameters::US_ANGLE_PITCH_5);

  m_acquisitionParameters.setTransmitFrequencyMax(transmitFrequencyMax);
  m_acquisitionParameters.setSamplingFrequencyMax(samplingFrequencyMax);
  m_acquisitionParameters.setImagingModeMax(imagingModeMax);
  m_acquisitionParameters.setImageDepthMax(imageDepthMax);
  m_acquisitionParameters.setSectorMax(sectorMax);
  m_acquisitionParameters.setMotorPositionMax(motorPositionMax);
  m_acquisitionParameters.setFramesPerVolumeMax(framesPerVolumeMax);
  if (anglePerFrame == 0)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_STATIC_MOTOR);
  else if (anglePerFrame == 2)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_ANGLE_PITCH_1);
  else if (anglePerFrame == 4)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_ANGLE_PITCH_2);
  else if (anglePerFrame == 8)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_ANGLE_PITCH_3);
  else if (anglePerFrame == 16)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_ANGLE_PITCH_4);
  else if (anglePerFrame == 32)
    m_acquisitionParameters.setSepsPerFrameMax(usAcquisitionParameters::US_ANGLE_PITCH_5);
}

/**
* Setter for motor activation : true to sweep the motor the motor during the acquisition, false to let it static.
*/
void usNetworkGrabber::setMotorActivation(bool activateMotor)
{
  m_acquisitionParameters.setActivateMotor(activateMotor);
}

/**
* Setter for steps between two sucessive frames (angle). See usAcquisitionParameters::usMotorSteps
*/
void usNetworkGrabber::setStepsPerFrame(usAcquisitionParameters::usMotorStep stepsPerFrame)
{
  m_acquisitionParameters.setSepsPerFrame(stepsPerFrame);
}

/**
* Setter for the number of frames per volume, in 3D acquisition.
*/
void usNetworkGrabber::setFramesPerVolume(int framesPerVolume)
{
  if (framesPerVolume < m_acquisitionParameters.getFramesPerVolumeMin() ||
      framesPerVolume > m_acquisitionParameters.getFramesPerVolumeMax() ||
      framesPerVolume % 2 == 0) // odd number of frames per volume required
    throw(vpException(vpException::badValue),
          "Number of frames per volume must be odd, and between min and max values");
  m_acquisitionParameters.setFramesPerVolume(framesPerVolume);
}

/**
* Setter for image depth (millimeters).
*/
void usNetworkGrabber::setImageDepth(int imageDepth)
{
  if (imageDepth < m_acquisitionParameters.getImageDepthMin() || imageDepth > m_acquisitionParameters.getImageDepthMax())
    throw(vpException(vpException::badValue), "Image depth must be included between min and max values");
  m_acquisitionParameters.setImageDepth(imageDepth);
}

/**
* Setter for imaging mode (0 : B-Mode, 12 : RF).
*/
void usNetworkGrabber::setImagingMode(int imagingMode)
{
  if (imagingMode < m_acquisitionParameters.getImagingModeMin() ||
      imagingMode > m_acquisitionParameters.getImagingModeMax())
    throw(vpException(vpException::badValue), "Imaging mode out of range");
  m_acquisitionParameters.setImagingMode(imagingMode);
}

/**
* Setter for a static motor position (in angle from the beginning position).
*/
void usNetworkGrabber::setMotorPosition(int motorPosition)
{
  if (motorPosition < m_acquisitionParameters.getMotorPositionMin() ||
      motorPosition > m_acquisitionParameters.getMotorPositionMax())
    throw(vpException(vpException::badValue), "Motor poisition out of range");
  m_acquisitionParameters.setMotorPosition(motorPosition);
}

/**
* Setter for post-scan image height.
*/
void usNetworkGrabber::setPostScanHeigh(int postScanHeigh) { m_acquisitionParameters.setPostScanHeigh(postScanHeigh); }

/**
* Setter for post-scan mode : true for post-scan, false for pre-scan.
*/
void usNetworkGrabber::setPostScanMode(bool postScanMode) { m_acquisitionParameters.setPostScanMode(postScanMode); }

/**
* Setter for post-scan image width.
*/
void usNetworkGrabber::setPostScanWidth(int postScanWidth) { m_acquisitionParameters.setPostScanWidth(postScanWidth); }

/**
* Setter for samplingFrequency (Hz).
*/
void usNetworkGrabber::setSamplingFrequency(int samplingFrequency)
{
  if (samplingFrequency < m_acquisitionParameters.getSamplingFrequencyMin() ||
      samplingFrequency > m_acquisitionParameters.getSamplingFrequencyMax())
    throw(vpException(vpException::badValue), "Sampling frequency out of range");
  m_acquisitionParameters.setSamplingFrequency(samplingFrequency);
}

/**
* Setter for sector (percentage of the number of transducers to use).
*/
void usNetworkGrabber::setSector(int sector)
{
  if (sector < m_acquisitionParameters.getSectorMin() || sector > m_acquisitionParameters.getSectorMax())
    throw(vpException(vpException::badValue), "Sector out of range");
  m_acquisitionParameters.setSector(sector);
}

/**
* Setter for transmitFrequency (Hz).
*/
void usNetworkGrabber::setTransmitFrequency(int transmitFrequency)
{
  if (transmitFrequency < m_acquisitionParameters.getTransmitFrequencyMin() ||
      transmitFrequency > m_acquisitionParameters.getTransmitFrequencyMax())
    throw(vpException(vpException::badValue), "Transmit frequency out of range");

  m_acquisitionParameters.setTransmitFrequency(transmitFrequency);
}

/**
* Sends the command to run the acquisition on the ultrasound station and start a new thread to run the data reception
*/
void usNetworkGrabber::runAcquisition()
{
  if (m_thread == NULL) {
    m_thread = new QThread;
    this->moveToThread(m_thread);
    m_thread->start();
  }
  emit runAcquisitionSignal(true);

  //vpTime::wait(10); // workaround to allow calling run / stop methods one just after another
}

/**
* Sends the command to stop the acquisition on the ulstrasound station.
* The server will stop to send data, but the grabber is still connected : you can then perform a runAcquisition() to
* tell the server to restart sending frames.
* The acquisition parameters will be kept.
*/
void usNetworkGrabber::stopAcquisition()
{
  emit runAcquisitionSignal(false);

  vpTime::wait(10); // workaround to allow calling run / stop methods one just after another
}

/**
  * Slot called to write on the socket the command run / stop.
  * @param run Boolean to run the acquisition (true), or stop it (false).
  */
void usNetworkGrabber::sendRunSignal(bool run)
{
  usRunControlHeaderSent header;
  header.run = run;

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif defined(USTK_HAVE_VTK_QT4)
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError, "your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)
  out << header.headerId;
  out << header.run;
  m_tcpSocket->write(block);
  m_isRunning = run;
}

/**
* Getter for motor activation : true to sweep the motor the motor during the acquisition, false to let it static.
*/
bool usNetworkGrabber::getMotorActivation() { return m_acquisitionParameters.getActivateMotor(); }

/**
* Getter for steps between two sucessive frames (angle). See usAcquisitionParameters::usMotorSteps
*/
usAcquisitionParameters::usMotorStep usNetworkGrabber::getStepsPerFrame()
{
  return m_acquisitionParameters.getSepsPerFrame();
}

/**
* Getter for the number of frames per volume, in 3D acquisition.
*/
int usNetworkGrabber::getFramesPerVolume() { return m_acquisitionParameters.getFramesPerVolume(); }

/**
* Getter for image depth (meters).
*/
int usNetworkGrabber::getImageDepth() { return m_acquisitionParameters.getImageDepth(); }

/**
* Getter for imaging mode (0 : B-Mode, 12 : RF).
*/
int usNetworkGrabber::getImagingMode() { return m_acquisitionParameters.getImagingMode(); }

/**
* Getter for a static motor position (in angle from the beginning position).
*/
int usNetworkGrabber::getMotorPosition() { return m_acquisitionParameters.getMotorPosition(); }

/**
* Getter for post-scan image height.
*/
int usNetworkGrabber::getPostScanHeigh() { return m_acquisitionParameters.getPostScanHeigh(); }

/**
* Getter for post-scan mode : true for post-scan, false for pre-scan.
*/
bool usNetworkGrabber::getPostScanMode() { return m_acquisitionParameters.getPostScanMode(); }

/**
* Getter for post-scan image width.
*/
int usNetworkGrabber::getPostScanWidth() { return m_acquisitionParameters.getPostScanWidth(); }

/**
* Getter for samplingFrequency (Hz).
*/
int usNetworkGrabber::getSamplingFrequency() { return m_acquisitionParameters.getSamplingFrequency(); }

/**
* Getter for sector (percentage of the number of transducers to use).
*/
int usNetworkGrabber::getSector() { return m_acquisitionParameters.getSector(); }

/**
* Getter for transmitFrequency (Hz).
*/
int usNetworkGrabber::getTransmitFrequency() { return m_acquisitionParameters.getTransmitFrequency(); }

/**
* Sets the  motor position of 4DC7 probe to the middle.
*/
void usNetworkGrabber::center3DProbeMotor()
{
  setMotorPosition(40);
  sendAcquisitionParameters();
}

#endif
