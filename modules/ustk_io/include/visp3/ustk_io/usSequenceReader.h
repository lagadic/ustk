/****************************************************************************
*
* This file is part of the UsTk software.
* Copyright (C) 2014 by Inria. All rights reserved.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License ("GPL") as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
* See the file COPYING at the root directory of this source
* distribution for additional information about the GNU GPL.
*
* This software was developed at:
* INRIA Rennes - Bretagne Atlantique
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* France
* http://www.irisa.fr/lagadic
*
* If you have questions regarding the use of this file, please contact the
* authors at Alexandre.Krupa@inria.fr
*
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
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

#ifndef US_SEQUENCE_READER_H
#define US_SEQUENCE_READER_H

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

  /** file name of sequence settings file (ex : sequence.xml), images files names are deduced from informations contained in it.*/
  std::string m_sequenceFileName;
  std::string m_genericImageFileName;
  bool m_fileNameIsSet;

  /** Top know if the sequence is already open*/
  bool is_open;

public:

  usSequenceReader();

  ~usSequenceReader();

  //attributes getters/setters
  double getFrameRate() const {return m_frameRate;}

  void setSequenceFileName(const std::string &sequenceFileName);

  void setFirstFrameIndex(long firstIndex);
  void setLastFrameIndex(long lastIndex);

  void open(ImageType &image);
  //get images in grabber style
  void acquire(ImageType &image);
  //get image by its number in the sequence
  void getFrame(ImageType &image,int position);

  /*!
    \return true if the end of the sequence is reached.
  */
  inline bool end() {
    if (m_frameCount > m_lastFrame )
      return true;
    return false;
  }

};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Constructor.
*/
template<class ImageType>
usSequenceReader<ImageType>::usSequenceReader() : m_frame(), m_frameRate(0.0), m_firstFrame(0), m_firstFrameIsSet(false),
 m_lastFrame(0), m_lastFrameIsSet(false), m_frameCount(0), m_sequenceFileName(""),m_genericImageFileName(""),m_fileNameIsSet(false)
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
*/
template<class ImageType>
void usSequenceReader<ImageType>::setSequenceFileName(const std::string &sequenceFileName)
{
  m_sequenceFileName = sequenceFileName;
  m_fileNameIsSet = true;
}

/**
* Sequence first index setter.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setFirstFrameIndex(long firstIndex)
{
  m_firstFrame = firstIndex;
  m_firstFrameIsSet = true;
}

/**
* Sequence last index setter.
*/
template<class ImageType>
void usSequenceReader<ImageType>::setLastFrameIndex(long lastIndex)
{
  m_lastFrame = lastIndex;
  m_lastFrameIsSet =true;
}

/**
* Sequence opening for generic image type : not implemented.
*/
template<class ImageType>
void usSequenceReader<ImageType>::open(ImageType &image)
{
  throw(vpException(vpException::notImplementedError));
}

/**
* Sequence opening for usImageRF2D type.
*/
template<>
inline void usSequenceReader<usImageRF2D<unsigned char> >::open(usImageRF2D<unsigned char> &image)
{
  if(!m_fileNameIsSet)
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));

  usImageSettingsXmlParser xmlParser;
  xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(xmlParser.getSequenceStartNumber());
  setLastFrameIndex(xmlParser.getSequenceStopNumber());
  m_frameRate = xmlParser.getSequenceFrameRate();
  m_genericImageFileName = xmlParser.getImageFileName();

  //saving the settings for all the rf sequence
  m_frame.setImageSettings(xmlParser.getImagePreScanSettings());

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",m_firstFrame);
  std::string imageFileName = vpIoTools::getNameWE(m_genericImageFileName) + buffer + vpIoTools::getFileExtension(m_genericImageFileName);
  std::cout << imageFileName << std::endl;
  vpImageIo::read(image,imageFileName);
  image.setImageSettings(m_frame);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

/**
* Sequence opening for usImagePreScan2D type.
*/
template<>
inline void usSequenceReader<usImagePreScan2D<unsigned char> >::open(usImagePreScan2D<unsigned char> &image)
{
  if(!m_fileNameIsSet)
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));

  usImageSettingsXmlParser xmlParser;
  xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(xmlParser.getSequenceStartNumber());
  setLastFrameIndex(xmlParser.getSequenceStopNumber());
  m_frameRate = xmlParser.getSequenceFrameRate();
  m_genericImageFileName = xmlParser.getImageFileName();

  //saving the settings for all the pre-scan sequence
  m_frame.setImageSettings(xmlParser.getImagePreScanSettings());

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",m_firstFrame);
  std::string imageFileName = vpIoTools::getNameWE(m_genericImageFileName) + buffer + vpIoTools::getFileExtension(m_genericImageFileName);
  std::cout << imageFileName << std::endl;
  vpImageIo::read(image,imageFileName);
  image.setImageSettings(m_frame);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

/**
* Sequence opening for usImagePreScan2D type.
*/
template<>
inline void usSequenceReader<usImagePostScan2D<unsigned char> >::open(usImagePostScan2D<unsigned char> &image)
{
  if(!m_fileNameIsSet)
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));

  usImageSettingsXmlParser xmlParser;
  xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(xmlParser.getSequenceStartNumber());
  setLastFrameIndex(xmlParser.getSequenceStopNumber());
  m_frameRate = xmlParser.getSequenceFrameRate();
  m_genericImageFileName = xmlParser.getImageFileName();

  //saving the settings for all the post scan sequence
  m_frame.setImageSettings(xmlParser.getImagePostScanSettings());

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",m_firstFrame);
  std::string imageFileName = vpIoTools::getNameWE(m_genericImageFileName) + buffer + vpIoTools::getFileExtension(m_genericImageFileName);
  std::cout << imageFileName << std::endl;
  vpImageIo::read(image,imageFileName);
  image.setImageSettings(m_frame);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

/**
* Sequence last index setter.
*/
template<class ImageType>
void usSequenceReader<ImageType>::acquire(ImageType &image)
{
  if(!is_open)
    open(image);

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",m_frameCount);
  std::string imageFileName = vpIoTools::getNameWE(m_genericImageFileName) + buffer + vpIoTools::getFileExtension(m_genericImageFileName);

  vpImageIo::read(image,imageFileName);
  image.setImageSettings(m_frame);

  m_frameCount++;
  is_open = true;
}

/**
* Sequence last index setter.
*/
template<class ImageType>
void usSequenceReader<ImageType>::getFrame(ImageType &image,int position)
{
  if(!is_open)
    is_open(image);

  if(position < m_firstFrame || position > m_lastFrame) {
    throw(vpException(vpException::badValue,"position out of range"));
  }

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",position);
  std::string imageFileName = vpIoTools::getNameWE(m_genericImageFileName) + buffer + vpIoTools::getFileExtension(m_genericImageFileName);

  vpImageIo::read(image,imageFileName);
  image.setImageSettins(m_frame);
}

#endif //US_SEQUENCE_READER_H
