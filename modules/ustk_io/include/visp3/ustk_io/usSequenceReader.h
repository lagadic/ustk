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
* @file usSequenceReader.h
* @brief Reading of sequences of ultrasound images
*
* This class is used to read ultrasound images from a sequence.
*/

#ifndef __usSequenceReader_h_
#define __usSequenceReader_h_

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usImageSettingsXmlParser.h>

/**
* @class usSequenceReader
* @brief Reading of sequences of ultrasound images
* @ingroup module_ustk_io
*
* This class is used to grab ultrasound images from a sequence.
*/
template <class ImageType>
class usSequenceReader
{
private:


  /** Ultrasound image settings saved for all the sequence reading*/
  ImageType m_frame;

  /** Sequence frameRate */
  double m_frameRate;

  /** First frame index*/
  long m_firstFrame;
  bool m_firstFrameIsSet;

  /** First frame index*/
  long m_lastFrame;
  bool m_lastFrameIsSet;

  /** Count the frame number when the class is used as a grabber. Incremented directly after grabbing an image (contains next index) */
  long m_frameCount;

  /** file name of sequence settings file (ex : sequence.xml), images files names are deduced from information contained in it.*/
  std::string m_sequenceFileName;
  std::string m_genericImageFileName;
  bool m_fileNameIsSet;

  /** Top know if the sequence is already open*/
  bool is_open;

  /** Loop cycling*/
  bool m_enableLoopCycling;
  int loopIncrement; // -1 or +1 depending on witch side we acquire

  /** Xml parser used*/
  usImageSettingsXmlParser m_xmlParser;

public:

  usSequenceReader();

  virtual ~usSequenceReader();

  //get images in grabber style
  void acquire(ImageType &image);


  /*!
    \return true if the end of the sequence is reached.
  */
  inline bool end() {
    if(!is_open) {
      open(m_frame);
      m_frameCount--;
    }
    //
    if(m_enableLoopCycling) {
      if(loopIncrement == 1) {
        if(m_frameCount > m_lastFrame ) {
          m_frameCount -= 2;
          loopIncrement = -1;
          return false;
        }
      }
      else if(loopIncrement == -1) {
        if(m_frameCount < m_firstFrame ) {
          m_frameCount += 2;
          loopIncrement = 1;
          return false;
        }
      }
    }
    else if (m_frameCount > m_lastFrame || m_frameCount < m_firstFrame )
      return true;
    return false;
  }

  //get image by its number in the sequence
  void getFrame(ImageType &image,int index);

  //get image by its number in the sequence
  ImageType& getFrame(int index);

  //total number of frames in the sequence
  long getFrameCount();

  //attributes getters/setters
  double getFrameRate() const {return m_frameRate;}
  long getImageNumber() const {return m_frameCount;}

  //get the xml parser : usefull to acess all the image settings contained in it
  usImageSettingsXmlParser getXmlParser();


  void open(ImageType &image);

  void setFirstFrameIndex(long firstIndex);
  void setLastFrameIndex(long lastIndex);
  void setLoopCycling(bool activateLoopCycling);
  void setSequenceFileName(const std::string &sequenceFileName);
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Default constructor.
*/
template<class ImageType>
usSequenceReader<ImageType>::usSequenceReader() : m_frame(), m_frameRate(0.0), m_firstFrame(0), m_firstFrameIsSet(false),
 m_lastFrame(0), m_lastFrameIsSet(false), m_frameCount(0), m_sequenceFileName(""),m_genericImageFileName(""), m_fileNameIsSet(false),
is_open(false), m_enableLoopCycling(false), loopIncrement(1)
{

}

/**
* Destructor.
*/
template<class ImageType>
usSequenceReader<ImageType>::~usSequenceReader()
{

}

/**
* Settings fileName setter.
* @param sequenceFileName Relative or absolute file name of the sequence header.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setSequenceFileName(const std::string &sequenceFileName)
{
  m_sequenceFileName = sequenceFileName;
  m_fileNameIsSet = true;
}

/**
* Sequence first index setter.
* @param firstIndex First index of the sequence.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setFirstFrameIndex(long firstIndex)
{
  m_firstFrame = firstIndex;
  m_firstFrameIsSet = true;
}

/**
* Sequence last index setter.
* @param lastIndex Last index of the sequence.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setLastFrameIndex(long lastIndex)
{
  m_lastFrame = lastIndex;
  m_lastFrameIsSet =true;
}

/**
* Sequence opening.
* @param image First image of the sequence to read.
*/
template<class ImageType>
void usSequenceReader<ImageType>::open(ImageType &image)
{
  (void) image;
  throw(vpException(vpException::notImplementedError));
}

template<>
void usSequenceReader<usImageRF2D<unsigned char> >::open(usImageRF2D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }
  m_xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(m_xmlParser.getSequenceStartNumber());
  setLastFrameIndex(m_xmlParser.getSequenceStopNumber());
  m_frameRate = m_xmlParser.getSequenceFrameRate();
  m_genericImageFileName = m_xmlParser.getImageFileName();

  //saving the settings for all the rf sequence
  m_frame.setTransducerRadius(m_xmlParser.getTransducerSettings().getTransducerRadius());
  m_frame.setScanLinePitch(m_xmlParser.getTransducerSettings().getScanLinePitch());
  m_frame.setScanLineNumber(m_xmlParser.getTransducerSettings().getScanLineNumber());
  m_frame.setTransducerConvexity(m_xmlParser.getTransducerSettings().isTransducerConvex());
  m_frame.setAxialResolution(m_xmlParser.getAxialResolution());

  //Reading image
  char buffer[FILENAME_MAX];
  sprintf(buffer, m_genericImageFileName.c_str(),m_firstFrame);
  std::string parentName = vpIoTools::getParent(m_sequenceFileName);
  if(!parentName.empty()) {
    parentName = parentName + vpIoTools::path("/");
  }
  std::string imageFileName =  parentName + buffer;
  vpImageIo::read(image,imageFileName);
  image.setImagePreScanSettings(m_frame);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

template<>
void usSequenceReader<usImagePreScan2D<unsigned char> >::open(usImagePreScan2D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }

  m_xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(m_xmlParser.getSequenceStartNumber());
  setLastFrameIndex(m_xmlParser.getSequenceStopNumber());
  m_frameRate = m_xmlParser.getSequenceFrameRate();
  m_genericImageFileName = m_xmlParser.getImageFileName();

  //saving the settings for all the pre-scan sequence
  m_frame.setTransducerRadius(m_xmlParser.getTransducerSettings().getTransducerRadius());
  m_frame.setScanLinePitch(m_xmlParser.getTransducerSettings().getScanLinePitch());
  m_frame.setScanLineNumber(m_xmlParser.getTransducerSettings().getScanLineNumber());
  m_frame.setTransducerConvexity(m_xmlParser.getTransducerSettings().isTransducerConvex());
  m_frame.setAxialResolution(m_xmlParser.getAxialResolution());

  //Reading image
  char buffer[FILENAME_MAX];
  sprintf(buffer, m_genericImageFileName.c_str(), m_firstFrame);
  std::string parentName = vpIoTools::getParent(m_sequenceFileName);
  if(!parentName.empty()) {
    parentName = parentName + vpIoTools::path("/");
  }
  std::string imageFileName =  parentName + buffer;
  vpImageIo::read(image,imageFileName);

  //workaround to prevent the resize by setImagePreScanSettings (having a scanline number at 0 if not precised in the xml)
  int scanlineNumber = image.getWidth();
  image.setImagePreScanSettings(m_frame);
  image.setScanLineNumber(scanlineNumber);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

template<>
void usSequenceReader<usImagePostScan2D<unsigned char> >::open(usImagePostScan2D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }

  m_xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(m_xmlParser.getSequenceStartNumber());
  setLastFrameIndex(m_xmlParser.getSequenceStopNumber());
  m_frameRate = m_xmlParser.getSequenceFrameRate();
  m_genericImageFileName = m_xmlParser.getImageFileName();

  //saving the settings for all the post scan sequence
  m_frame.setTransducerRadius(m_xmlParser.getTransducerSettings().getTransducerRadius());
  m_frame.setScanLinePitch(m_xmlParser.getTransducerSettings().getScanLinePitch());
  m_frame.setScanLineNumber(m_xmlParser.getTransducerSettings().getScanLineNumber());
  m_frame.setTransducerConvexity(m_xmlParser.getTransducerSettings().isTransducerConvex());
  m_frame.setWidthResolution(m_xmlParser.getWidthResolution());
  m_frame.setHeightResolution(m_xmlParser.getHeightResolution());

  //Reading image
  char buffer[FILENAME_MAX];
  sprintf(buffer, m_genericImageFileName.c_str(), m_firstFrame);
  std::string parentName = vpIoTools::getParent(m_sequenceFileName);
  if(!parentName.empty()) {
    parentName = parentName + vpIoTools::path("/");
  }
  std::string imageFileName =  parentName + buffer;
  vpImageIo::read(image,imageFileName);

  image.setTransducerRadius(m_frame.getTransducerRadius());
  image.setScanLinePitch(m_frame.getScanLinePitch());
  image.setScanLineNumber(m_frame.getScanLineNumber());
  image.setTransducerConvexity(m_frame.isTransducerConvex());
  image.setWidthResolution(m_frame.getWidthResolution());
  image.setHeightResolution(m_frame.getHeightResolution());

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

/**
* Sequence image acquisition (grabber-style : an internal counter is incremented to open next image at the next call).
* @param image Image of the sequence to read.
*/
template<class ImageType>
void usSequenceReader<ImageType>::acquire(ImageType &image)
{
  if (!is_open) {
    this->open(image);
    return;
  }
  //Reading image
  char buffer[FILENAME_MAX];
  sprintf(buffer, m_genericImageFileName.c_str(), m_frameCount);
  std::string parentName = vpIoTools::getParent(m_sequenceFileName);
  if(!parentName.empty()) {
    parentName = parentName + vpIoTools::path("/");
  }
  std::string imageFileName =  parentName + buffer;

  vpImageIo::read(image,imageFileName);
  image.setTransducerSettings(m_frame);
  image.setScanLineNumber(image.getWidth());

  m_frameCount+=loopIncrement;
}

/**
* Sequence image acquisition with selection of the index (bypassing the internal counter).
* @param image Image of the sequence to read.
* @param index Index of the image you want to acquire.
*/
template<class ImageType>
void usSequenceReader<ImageType>::getFrame(ImageType &image, int index)
{
  if (!is_open) {
    open(image);
    return;
  }
  if(index < m_firstFrame || index > m_lastFrame) {
    throw(vpException(vpException::badValue,"position out of range"));
  }

  //Reading image
  char buffer[FILENAME_MAX];
  sprintf(buffer, m_genericImageFileName.c_str(), m_firstFrame);
  std::string imageFileName = buffer;

  vpImageIo::read(image,imageFileName);
  image.setImageSettins(m_frame);
}

/**
* Sequence image acquisition with selection of the index (bypassing the internal counter).
* @return image Image read.
* @param index Index of the image you want to acquire.
*/
template<class ImageType>
ImageType& usSequenceReader<ImageType>::getFrame(int index) {
  ImageType image;
  getFrame(image, index);
  return image;
}

/**
* Activate loop cycling mode
* @param activateLoopCycling True if you want to activate it, false to stop the loop.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setLoopCycling(bool activateLoopCycling)
{
  m_enableLoopCycling = activateLoopCycling;
}

/**
* Get the total number of frames in the sequence.
* @return Total number of frames in the sequence.
*/
template<class ImageType>
long usSequenceReader<ImageType>::getFrameCount()
{
  if(is_open)
    return m_lastFrame - m_firstFrame;
  throw(vpException(vpException::fatalError, "Cannot compute the frame number of the sequence before opening it !"));
}

/**
* Get the xml parser used to access all the image settings written in the xml file.
* @return Xml parser used by the sequence reader.
*/
template<class ImageType>
usImageSettingsXmlParser usSequenceReader<ImageType>::getXmlParser()
{
  if(is_open)
    return m_xmlParser;
  throw(vpException(vpException::fatalError, "Sequence not opened, xml parser is empty !"));
}
#endif //US_SEQUENCE_READER_H
