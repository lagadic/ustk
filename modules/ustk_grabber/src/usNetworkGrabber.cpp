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

#if defined(USTK_GRABBER_HAVE_QT5)

#include <iostream>
#include <fstream>
#include <visp3/io/vpImageIo.h>

#include <QDataStream>
#include <QEventLoop>

using namespace std;

/**
* Constructor.
* @param parent Parent QObject.
*/
usNetworkGrabber::usNetworkGrabber(QObject *parent) :
  QObject(parent)
{
  m_ip = "192.168.100.2";

  m_tcpSocket = new QTcpSocket(this);

  m_bytesLeftToRead = 0;

  m_verbose = false;

  m_connect = false;

  m_acquisitionParamters = usAcquisitionParameters();
  m_confirmHeader = usInitHeaderConfirmation();
  m_imageHeader = usImageHeader();

  QObject::connect(this, SIGNAL(serverUpdateEnded(bool)), this, SLOT(serverUpdated(bool)));
}

/**
* Destructor.
*/
usNetworkGrabber::~usNetworkGrabber()
{
  if(m_tcpSocket->isOpen())
    m_tcpSocket->close();
}

/**
* Selection of the server ip adress : local loop (if simulation) or 192.168.100.2.
*/
void usNetworkGrabber::useSimulator(bool t_state)
{
  m_ip = (t_state)?"127.0.0.1":"192.168.100.2";
}

/**
* Method used to initialize / stop the grabber.
* @param actionConnect Boolean to initialize (if true) or stop (if false) the grabber.
*/
void usNetworkGrabber::setConnection(bool actionConnect)
{
  m_connect = actionConnect;
  if(actionConnect)
    this->connectToServer();
  else
    m_tcpSocket->disconnect();
}

/**
* Method used to connect / disconnect to the server and manage signals/slots communication.
*/
void usNetworkGrabber::connectToServer()
{
  if(m_connect)
  {
    if(m_verbose)
      std::cout << "ip listening : " << m_ip.c_str() << std::endl;
    QHostAddress addr(m_ip.c_str());
    m_tcpSocket->connectToHost(addr, 8080);

    connect(m_tcpSocket, SIGNAL(connected()), this, SLOT(connected()));
    connect(m_tcpSocket, SIGNAL(disconnected()), this, SLOT(disconnected()));
    connect(m_tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(handleError(QAbstractSocket::SocketError)));

    if (m_tcpSocket->isOpen ()) {
      if(m_verbose) {
        std::cout << "socket is open." << std::endl;}}
    else {
      if(m_verbose) {
        std::cout << "socket not is open." << std::endl;}}

    if ( m_tcpSocket->isReadable()) {
      if(m_verbose) {
        std::cout << "socket is readable." << std::endl;}}
  }
  else
    m_tcpSocket->close();
}

/**
* Slot called when the grabber is connected to the server. Prints connection informations.
*/
void usNetworkGrabber::connected()
{
  if(m_verbose) {
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
  if(m_verbose)
    std::cout <<"Disconnected .... \n";
  m_tcpSocket->close();
}

/**
* Slot called if there is an error (or disruption) in the connection. Throws an exception and prints the error.
*/
void usNetworkGrabber::handleError(QAbstractSocket::SocketError err)
{
  Q_UNUSED(err);
  // Notify an error. tcpSocket.errorString() automatically gets an error message (in english).
  throw(vpException(vpException::fatalError,m_tcpSocket->errorString().toStdString()));

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
bool usNetworkGrabber::initAcquisition(const usNetworkGrabber::usInitHeaderSent &header) {

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_GRABBER_HAVE_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)

  out << header.headerId;
  out << header.probeId;
  out << header.slotId;
  out << header.imagingMode;
  m_tcpSocket->write(block);

  if(m_verbose)
    std::cout << "INIT SENT, waiting server confirmation..." << std::endl;
  // loop to wait update on server
  QEventLoop * loop = new QEventLoop;
  QObject::connect(this, SIGNAL(endBlockingLoop()), loop, SLOT(quit()));
  loop->exec();

  if(m_verbose)
    std::cout << "server confirmation : " << (int)m_updateParametersSucess << std::endl;

  return m_updateParametersSucess;
}

/**
* Method to close the connection.
*/
void usNetworkGrabber::disconnect() {
  m_tcpSocket->disconnect();
}

/**
* Method to send to the server the new acquisition parameters.
*/
bool usNetworkGrabber::sendAcquisitionParameters() {
  QByteArray block;
  QDataStream stream(&block, QIODevice::WriteOnly);
#if (defined(USTK_GRABBER_HAVE_QT5))
  stream.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  stream#else
      throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)

  stream << 2; // update id = 2
  stream << m_acquisitionParamters.getTransmitFrequency();
  stream << m_acquisitionParamters.getSamplingFrequency();
  stream << m_acquisitionParamters.getImagingMode();
  stream << m_acquisitionParamters.getPostScanMode();
  stream << m_acquisitionParamters.getPostScanHeigh();
  stream << m_acquisitionParamters.getPostScanWidth();
  stream << m_acquisitionParamters.getImageDepth();
  stream << m_acquisitionParamters.getSector();
  stream << m_acquisitionParamters.getActivateMotor();
  stream << m_acquisitionParamters.getMotorPosition();
  stream << m_acquisitionParamters.getFramesPerVolume();
  stream << m_acquisitionParamters.getDegreesPerFrame();

  if(m_verbose) {
    std::cout << "UPDATE SENT : " << std::endl;
    std::cout << "TransmitFrequency = " <<  m_acquisitionParamters.getTransmitFrequency() << std::endl;
    std::cout << "SamplingFrequency = " <<  m_acquisitionParamters.getSamplingFrequency() << std::endl;
    std::cout << "ImagingMode = " <<  m_acquisitionParamters.getImagingMode() << std::endl;
    std::cout << "PostScanMode = " <<  m_acquisitionParamters.getPostScanMode() << std::endl;
    std::cout << "PostScanHeigh = " <<  m_acquisitionParamters.getPostScanHeigh() << std::endl;
    std::cout << "PostScanWidth = " <<  m_acquisitionParamters.getPostScanWidth() << std::endl;
    std::cout << "ImageDepth = " <<  m_acquisitionParamters.getImageDepth() << std::endl;
    std::cout << "Sector = " <<  m_acquisitionParamters.getSector() << std::endl;
    std::cout << "ActivateMotor = " <<  m_acquisitionParamters.getActivateMotor() << std::endl;
    std::cout << "MotorPosition = " <<  m_acquisitionParamters.getMotorPosition() << std::endl;
    std::cout << "FramesPerVolume = " <<  m_acquisitionParamters.getFramesPerVolume() << std::endl;
    std::cout << "DegreesPerFrame = " <<  m_acquisitionParamters.getDegreesPerFrame() << std::endl;
  }
  m_tcpSocket->write(block);

  if(m_verbose)
    std::cout << "waiting server confirmation..." << std::endl;

  // loop to wait update on server side
  QEventLoop * loop = new QEventLoop;
  QObject::connect(this, SIGNAL(endBlockingLoop()), loop, SLOT(quit()));
  loop->exec();

  if(m_verbose)
    std::cout << "Update server confirmation : " << (int)m_updateParametersSucess << std::endl;

  return m_updateParametersSucess;
}

/**
* Slot called when we get the answer from the server to our reques to update aquisition parameters.
*/
void usNetworkGrabber::serverUpdated(bool sucess) {
  m_updateParametersSucess = sucess;
  emit(endBlockingLoop());
}

/**
* Method to read all parameters comming from the server (in answer to an update). It fills the usNetworkGrabber attribute.
*/
void usNetworkGrabber::readAcquisitionParameters(QDataStream &stream) {

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
  int degreesPerFrame;
  int transmitFrequencyMin;
  int samplingFrequencyMin;
  int imagingModeMin;
  int imageDepthMin;
  int sectorMin;
  int motorPositionMin;
  int framesPerVolumeMin;
  int degreesPerFrameMin;
  int transmitFrequencyMax;
  int samplingFrequencyMax;
  int imagingModeMax;
  int imageDepthMax;
  int sectorMax;
  int motorPositionMax;
  int framesPerVolumeMax;
  int degreesPerFrameMax;

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
  stream >> degreesPerFrame;
  stream >> transmitFrequencyMin;
  stream >> samplingFrequencyMin;
  stream >> imagingModeMin;
  stream >> imageDepthMin;
  stream >> sectorMin;
  stream >> motorPositionMin;
  stream >> framesPerVolumeMin;
  stream >> degreesPerFrameMin;
  stream >> transmitFrequencyMax;
  stream >> samplingFrequencyMax;
  stream >> imagingModeMax;
  stream >> imageDepthMax;
  stream >> sectorMax;
  stream >> motorPositionMax;
  stream >> framesPerVolumeMax;
  stream >> degreesPerFrameMax;

  if(m_verbose) {
    std::cout << "transmitFrequency = " <<transmitFrequency << std::endl;
    std::cout << "samplingFrequency = " <<samplingFrequency << std::endl;
    std::cout << "imagingMode = " <<imagingMode << std::endl;
    std::cout << "postScanMode = " <<postScanMode << std::endl;
    std::cout << "postScanHeigh = " <<postScanHeigh << std::endl;
    std::cout << "postScanWidth = " <<postScanWidth << std::endl;
    std::cout << "imageDepth = " <<imageDepth << std::endl;
    std::cout << "sector = " <<sector << std::endl;
    std::cout << "activateMotor = " <<activateMotor << std::endl;
    std::cout << "motorPosition = " <<motorPosition << std::endl;
    std::cout << "framesPerVolume = " <<framesPerVolume << std::endl;
    std::cout << "degreesPerFrame = " <<degreesPerFrame << std::endl;
    std::cout << "transmitFrequencyMin = " <<transmitFrequencyMin << std::endl;
    std::cout << "samplingFrequencyMin = " <<samplingFrequencyMin << std::endl;
    std::cout << "imagingModeMin = " <<imagingModeMin << std::endl;
    std::cout << "imageDepthMin = " <<imageDepthMin << std::endl;
    std::cout << "sectorMin = " <<sectorMin << std::endl;
    std::cout << "motorPositionMin = " <<motorPositionMin << std::endl;
    std::cout << "framesPerVolumeMin = " <<framesPerVolumeMin << std::endl;
    std::cout << "degreesPerFrameMin = " <<degreesPerFrameMin << std::endl;
    std::cout << "transmitFrequencyMax = " <<transmitFrequencyMax << std::endl;
    std::cout << "samplingFrequencyMax = " <<samplingFrequencyMax << std::endl;
    std::cout << "imagingModeMax = " <<imagingModeMax << std::endl;
    std::cout << "imageDepthMax = " <<imageDepthMax << std::endl;
    std::cout << "sectorMax = " <<sectorMax << std::endl;
    std::cout << "motorPositionMax = " <<motorPositionMax << std::endl;
    std::cout << "framesPerVolumeMax = " <<framesPerVolumeMax << std::endl;
    std::cout << "degreesPerFrameMax = " <<degreesPerFrameMax << std::endl;
  }

  m_acquisitionParamters.setTransmitFrequency(transmitFrequency);
  m_acquisitionParamters.setSamplingFrequency(samplingFrequency);
  m_acquisitionParamters.setImagingMode(imagingMode);
  m_acquisitionParamters.setPostScanMode(postScanMode);
  m_acquisitionParamters.setPostScanHeigh(postScanHeigh);
  m_acquisitionParamters.setPostScanWidth(postScanWidth);
  m_acquisitionParamters.setImageDepth(imageDepth);
  m_acquisitionParamters.setSector(sector);
  m_acquisitionParamters.setActivateMotor(activateMotor);
  m_acquisitionParamters.setMotorPosition(motorPosition);
  m_acquisitionParamters.setFramesPerVolume(framesPerVolume);
  m_acquisitionParamters.setDegreesPerFrame(degreesPerFrame);

  m_acquisitionParamters.setTransmitFrequencyMin(transmitFrequencyMin);
  m_acquisitionParamters.setSamplingFrequencyMin(samplingFrequencyMin);
  m_acquisitionParamters.setImagingModeMin(imagingModeMin);
  m_acquisitionParamters.setImageDepthMin(imageDepthMin);
  m_acquisitionParamters.setSectorMin(sectorMin);
  m_acquisitionParamters.setMotorPositionMin(motorPositionMin);
  m_acquisitionParamters.setFramesPerVolumeMin(framesPerVolumeMin);
  m_acquisitionParamters.setDegreesPerFrameMin(degreesPerFrameMin);

  m_acquisitionParamters.setTransmitFrequencyMax(transmitFrequencyMax);
  m_acquisitionParamters.setSamplingFrequencyMax(samplingFrequencyMax);
  m_acquisitionParamters.setImagingModeMax(imagingModeMax);
  m_acquisitionParamters.setImageDepthMax(imageDepthMax);
  m_acquisitionParamters.setSectorMax(sectorMax);
  m_acquisitionParamters.setMotorPositionMax(motorPositionMax);
  m_acquisitionParamters.setFramesPerVolumeMax(framesPerVolumeMax);
  m_acquisitionParamters.setDegreesPerFrameMax(degreesPerFrameMax);
}

/**
* Setter for motor activation : true to sweep the motor the motor during the acquisition, false to let it static.
*/
void usNetworkGrabber::setActivateMotor(bool activateMotor) {
  m_acquisitionParamters.setActivateMotor(activateMotor);
}

/**
* Setter for angle between two sucessive frames (degrees).
*/
void usNetworkGrabber::setDegreesPerFrame(int degreesPerFrame) {
  if(degreesPerFrame < m_acquisitionParamters.getDegreesPerFrameMin() ||
     degreesPerFrame > m_acquisitionParamters.getDegreesPerFrameMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setDegreesPerFrame(degreesPerFrame);
}


/**
* Setter for the number of frames per volume, in 3D acquisition.
*/
void usNetworkGrabber::setFramesPerVolume(int framesPerVolume) {
  if(framesPerVolume < m_acquisitionParamters.getFramesPerVolumeMin() ||
     framesPerVolume > m_acquisitionParamters.getFramesPerVolumeMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setFramesPerVolume(framesPerVolume);
}

/**
* Setter for image depth (meters).
*/
void usNetworkGrabber::setImageDepth(int imageDepth) {
  if(imageDepth < m_acquisitionParamters.getImageDepthMin() ||
     imageDepth > m_acquisitionParamters.getImageDepthMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setImageDepth(imageDepth);
}

/**
* Setter for imaging mode (0 : B-Mode, 12 : RF).
*/
void usNetworkGrabber::setImagingMode(int imagingMode) {
  if(imagingMode < m_acquisitionParamters.getImagingModeMin() ||
     imagingMode > m_acquisitionParamters.getImagingModeMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setImagingMode(imagingMode);
}

/**
* Setter for a static motor position (in degrees from the beginning position).
*/
void usNetworkGrabber::setMotorPosition(int motorPosition) {
  if(motorPosition < m_acquisitionParamters.getMotorPositionMin() ||
     motorPosition > m_acquisitionParamters.getMotorPositionMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setMotorPosition(motorPosition);
}

/**
* Setter for post-scan image height.
*/
void usNetworkGrabber::setPostScanHeigh(int postScanHeigh) {
  m_acquisitionParamters.setPostScanHeigh(postScanHeigh);
}

/**
* Setter for post-scan mode : true for post-scan, false for pre-scan.
*/
void usNetworkGrabber::setPostScanMode(bool postScanMode) {
  m_acquisitionParamters.setPostScanMode(postScanMode);
}

/**
* Setter for post-scan image width.
*/
void usNetworkGrabber::setPostScanWidth(int postScanWidth) {
  m_acquisitionParamters.setPostScanWidth(postScanWidth);
}

/**
* Setter for samplingFrequency (Hz).
*/
void usNetworkGrabber::setSamplingFrequency(int samplingFrequency) {
  if(samplingFrequency < m_acquisitionParamters.getSamplingFrequencyMin() ||
     samplingFrequency > m_acquisitionParamters.getSamplingFrequencyMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setSamplingFrequency(samplingFrequency);
}

/**
* Setter for sector (percentage of the number of transducers to use).
*/
void usNetworkGrabber::setSector(int sector) {
  if(sector < m_acquisitionParamters.getSectorMin() ||
     sector > m_acquisitionParamters.getSectorMax())
    throw(vpException(vpException::badValue));
  m_acquisitionParamters.setSector(sector);
}

/**
* Setter for transmitFrequency (Hz).
*/
void usNetworkGrabber::setTransmitFrequency(int transmitFrequency) {
  if(transmitFrequency < m_acquisitionParamters.getTransmitFrequencyMin() ||
     transmitFrequency > m_acquisitionParamters.getTransmitFrequencyMax())
    throw(vpException(vpException::badValue));

  m_acquisitionParamters.setTransmitFrequency(transmitFrequency);
}

/**
* Sends the command to run the acquisition on the ulstrasound station.
*/
void usNetworkGrabber::runAcquisition() {

  usRunControlHeaderSent header;
  header.run = true;

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_GRABBER_HAVE_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)
  out << header.headerId;
  out << header.run;
  m_tcpSocket->write(block);
}

/**
* Sends the command to stop the acquisition on the ulstrasound station.
* The server will stop to send data, but the grabber is still connected : you can then perform a runAcquisition() to tell the server to restart sending frames.
* The acquisition parameters will be kept.
*/
void usNetworkGrabber::stopAcquisition() {
  usRunControlHeaderSent header;
  header.run = false;

  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
#if (defined(USTK_GRABBER_HAVE_QT5))
  out.setVersion(QDataStream::Qt_5_0);
#elif (defined(USTK_HAVE_QT4))
  out.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  // Writing on the stream. Warning : order matters ! (must be the same as on server side when reading)
  out << header.headerId;
  out << header.run;
  m_tcpSocket->write(block);
}

#endif
