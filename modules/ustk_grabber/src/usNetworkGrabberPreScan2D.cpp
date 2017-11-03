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

#include <visp3/ustk_grabber/usNetworkGrabberPreScan2D.h>

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <QtCore/QDataStream>
#include <QtCore/QEventLoop>
#include <visp3/ustk_io/usImageIo.h>

/**
* Constructor. Inititializes the image, and manages Qt signal.
*/
usNetworkGrabberPreScan2D::usNetworkGrabberPreScan2D(usNetworkGrabber *parent) :
  usNetworkGrabber(parent)
{
  m_grabbedImage.init(0,0);

  //buffer of size 3
  m_outputBuffer.push_back(new usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >);
  m_outputBuffer.push_back(new usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >);
  m_outputBuffer.push_back(new usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >);

  m_firstFrameAvailable = false;

  m_swichOutputInit = false;

  m_recordingOn = false;

  connect(m_tcpSocket ,SIGNAL(readyRead()),this, SLOT(dataArrived()));
}


/**
* Destructor.
*/
usNetworkGrabberPreScan2D::~usNetworkGrabberPreScan2D()
{

}

/**
* Slot called when data is coming on the network.
* Manages the type of data which is coming and read it. Emits newFrameArrived signal when a whole frame is available.
*/
void usNetworkGrabberPreScan2D::dataArrived()
{
  ////////////////// HEADER READING //////////////////
  QDataStream in;
  in.setDevice(m_tcpSocket);
#if (defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT5))
  in.setVersion(QDataStream::Qt_5_0);
#elif defined(USTK_HAVE_VTK_QT4)
  in.setVersion(QDataStream::Qt_4_8);
#else
  throw(vpException(vpException::fatalError,"your Qt version is not managed in ustk"));
#endif

  int headerType;
  if(m_bytesLeftToRead == 0 ) { // do not try to read a header if last frame was not complete
    in >> headerType;
    if(m_verbose)
      std::cout << "header received, type = " << headerType << std::endl;
  }
  else {
    headerType = 0; // not a header received, but a part of a frame
  }
  //init confirm header received
  if(headerType == m_confirmHeader.headerId) {
    //read whole header
    in >> m_confirmHeader.initOk;
    in >> m_confirmHeader.probeId;

    if(m_confirmHeader.initOk == 0) {
      m_tcpSocket->close();
      throw(vpException(vpException::fatalError, "porta initialisation error closing connection."));
    }
    if(m_verbose)
      std::cout << "porta init sucess, detected probe id = " << m_confirmHeader.probeId << std::endl;

    //read all acquisition parameters received
    readAcquisitionParameters(in);

    emit(serverUpdateEnded(m_confirmHeader.initOk));
  }

  //image header received
  else if(headerType == m_imageHeader.headerId) {
    //read whole header
    in >> m_imageHeader.frameCount;
    quint64 timestamp;
    in >> timestamp;
    m_imageHeader.timeStamp = timestamp;
    in >> m_imageHeader.dataRate;
    in >> m_imageHeader.dataLength;
    in >> m_imageHeader.ss;
    in >> m_imageHeader.imageType;
    in >> m_imageHeader.frameWidth;
    in >> m_imageHeader.frameHeight;
    in >> m_imageHeader.pixelWidth;
    in >> m_imageHeader.pixelHeight;
    in >> m_imageHeader.transmitFrequency;
    in >> m_imageHeader.samplingFrequency;

    in >> m_imageHeader.transducerRadius;
    in >> m_imageHeader.scanLinePitch;
    in >> m_imageHeader.scanLineNumber;
    in >> m_imageHeader.imageDepth;
    in >> m_imageHeader.anglePerFr;
    in >> m_imageHeader.framesPerVolume;
    in >> m_imageHeader.motorRadius;
    in >> m_imageHeader.motorType;

    if(m_verbose) {
      std::cout << "frameCount = " <<  m_imageHeader.frameCount << std::endl;
      std::cout << "timeStamp = " <<  m_imageHeader.timeStamp << std::endl;
      std::cout << "dataRate = " <<  m_imageHeader.dataRate << std::endl;
      std::cout << "dataLength = " <<  m_imageHeader.dataLength << std::endl;
      std::cout << "ss = " <<  m_imageHeader.ss << std::endl;
      std::cout << "imageType = " <<  m_imageHeader.imageType << std::endl;
      std::cout << "frameWidth = " <<  m_imageHeader.frameWidth << std::endl;
      std::cout << "frameHeight = " <<  m_imageHeader.frameHeight << std::endl;
      std::cout << "pixelWidth = " <<  m_imageHeader.pixelWidth << std::endl;
      std::cout << "pixelHeight = " <<  m_imageHeader.pixelHeight << std::endl;
      std::cout << "transmitFrequency = " <<  m_imageHeader.transmitFrequency << std::endl;
      std::cout << "samplingFrequency = " <<  m_imageHeader.samplingFrequency << std::endl;
      std::cout << "transducerRadius = " <<  m_imageHeader.transducerRadius << std::endl;
      std::cout << "scanLinePitch = " <<  m_imageHeader.scanLinePitch << std::endl;
      std::cout << "scanLineNumber = " <<  m_imageHeader.scanLineNumber << std::endl;
      std::cout << "imageDepth = " <<  m_imageHeader.imageDepth << std::endl;
      std::cout << "anglePerFr = " <<  m_imageHeader.anglePerFr << std::endl;
      std::cout << "framesPerVolume = " <<  m_imageHeader.framesPerVolume << std::endl;
      std::cout << "motorRadius = " <<  m_imageHeader.motorRadius << std::endl;
      std::cout << "motorType = " <<  m_imageHeader.motorType << std::endl;
    }

    //update transducer settings with image header received
    m_grabbedImage.setTransducerRadius(m_imageHeader.transducerRadius);
    m_grabbedImage.setScanLinePitch(m_imageHeader.scanLinePitch);
    m_grabbedImage.setDepth(m_imageHeader.imageDepth / 1000.0);
    m_grabbedImage.setTransducerConvexity(m_imageHeader.transducerRadius != 0.);
    m_grabbedImage.setTransmitFrequency(m_imageHeader.transmitFrequency);
    m_grabbedImage.setSamplingFrequency(m_imageHeader.samplingFrequency);

    //set data info
    m_grabbedImage.setFrameCount(m_imageHeader.frameCount);
    m_grabbedImage.setFramesPerVolume(m_imageHeader.framesPerVolume);
    m_grabbedImage.setTimeStamp(m_imageHeader.timeStamp);

    //warning if timestamps are close (< 1 ms)
    if (m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->getTimeStamp() -
        m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC)->getTimeStamp() < 1) {
          std::cout << "WARNING : new image received with an acquisition timestamp close to previous image" << std::endl;
        }

    m_grabbedImage.resize(m_imageHeader.frameWidth,m_imageHeader.frameHeight);

    m_bytesLeftToRead = m_imageHeader.dataLength;

    m_bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap,m_imageHeader.dataLength);

    if(m_bytesLeftToRead == 0 ) { // we've read all the frame in 1 packet.
      invertRowsCols();
    }
    if(m_verbose)
      std::cout << "Bytes left to read for whole frame = " << m_bytesLeftToRead << std::endl;

  }

  //we have a part of the image still not read (arrived with next tcp packet)
  else {
    if(m_verbose) {
      std::cout << "reading following part of the frame" << std::endl;
      std::cout << "local image size = " << m_grabbedImage.getSize() << std::endl;
    }
    m_bytesLeftToRead -= in.readRawData((char*)m_grabbedImage.bitmap+(m_grabbedImage.getSize()-m_bytesLeftToRead),m_bytesLeftToRead);

    if(m_bytesLeftToRead==0) { // we've read the last part of the frame.
      invertRowsCols();
    }
  }
}

/**
* Method to invert rows and columns in the image.
*/
void usNetworkGrabberPreScan2D::invertRowsCols() {
  // At this point, CURRENT_FILLED_FRAME_POSITION_IN_VEC is going to be filled
  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->setTransducerSettings(m_grabbedImage);

  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->setFrameCount(m_grabbedImage.getFrameCount());
  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->setFramesPerVolume(m_grabbedImage.getFramesPerVolume());
  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->setTimeStamp(m_grabbedImage.getTimeStamp());

  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC)->resize(m_grabbedImage.getWidth(),m_grabbedImage.getHeight());

  for(unsigned int i=0; i<m_grabbedImage.getHeight(); i++)
    for (unsigned int j=0; j<m_grabbedImage.getWidth(); j++)
      (*m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC))(j,i,m_grabbedImage(i,j));

  // Now CURRENT_FILLED_FRAME_POSITION_IN_VEC has become the last frame received
  // So we switch pointers beween MOST_RECENT_FRAME_POSITION_IN_VEC and CURRENT_FILLED_FRAME_POSITION_IN_VEC
  usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >* savePtr = m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC);
  m_outputBuffer.at(CURRENT_FILLED_FRAME_POSITION_IN_VEC) = m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC);
  m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC) = savePtr;
  if(m_recordingOn)
    m_sequenceWriter.write(*m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC),m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC)->getTimeStamp());

  m_firstFrameAvailable = true;
  emit(newFrameAvailable());
}

/**
* Method to get the last frame received. The grabber is designed to avoid data copy (it is why you get a pointer on the data).
* @note This method is designed to be thread-safe, you can call it from another thread.
* @return Pointer to the last frame acquired.
*/
usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >* usNetworkGrabberPreScan2D::acquire() {
  //check if the first frame is arrived
  if (!m_firstFrameAvailable) {
    throw(vpException(vpException::fatalError, "first frame not yet grabbed, cannot acquire"));
  }

  //user grabs too fast
  if(m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC)->getFrameCount() == m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC)->getFrameCount() + 1) {
    //we wait until a new frame is available
    QEventLoop loop;
    loop.connect(this, SIGNAL(newFrameAvailable()), SLOT(quit()));
    loop.exec();

    //switch pointers
    usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >* savePtr = m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC);
    m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC) = m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC);
    m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC) = savePtr;
    m_swichOutputInit = true;
  }

  // if more recent frame available
  else if(m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC)->getFrameCount() < m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC)->getFrameCount() || !m_swichOutputInit) {
    //switch pointers (output <-> mostRecentFilled)
    usFrameGrabbedInfo<usImagePreScan2D<unsigned char> >* savePtr = m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC);
    m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC) = m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC);
    m_outputBuffer.at(MOST_RECENT_FRAME_POSITION_IN_VEC) = savePtr;
    m_swichOutputInit = true;
  }
  return m_outputBuffer.at(OUTPUT_FRAME_POSITION_IN_VEC);
}

/**
* Method to link every image of the internal buffer to the vpDisplay you want to use. To call before any display operation using vpDisplay.
* @param display The vpDisplay used to display your images.
*/
void usNetworkGrabberPreScan2D::useVpDisplay(vpDisplay * display) {
  for(unsigned int i=0; i<m_outputBuffer.size(); i++)
    m_outputBuffer.at(i)->display = display;
}

/**
* Method to record the sequence received, to replay it later with the virtual server for example.
* @param path The path where the sequence will be saved.
*/
void usNetworkGrabberPreScan2D::activateRecording(std::string path) {
  m_recordingOn = true;
  m_sequenceWriter.setSequenceDirectory(path);
}

/**
* Stop recording process.
*/
void usNetworkGrabberPreScan2D::stopRecording() {
  m_recordingOn = false;
}

#endif
