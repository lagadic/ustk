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
* @file usSequenceWriter.h
* @brief Writing of sequences of ultrasound images.
*
* This class is used to write multiple ultrasound images.
*/

#ifndef US_SEQUENCE_WRITER_H
#define US_SEQUENCE_WRITER_H

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
* @class usSequenceWriter
* @brief Writing of sequences of ultrasound images.
* @ingroup module_ustk_io
*
* This class is used to write ultrasound images from a sequence.
*/
template <class ImageType>
class usSequenceWriter
{
public:

  usSequenceWriter();

  ~usSequenceWriter();

  //attributes getters/setters
  void setFrameRate(double frameRate);
  double getFrameRate() const;

  void setSequenceFileName(const std::string &sequenceFileName);
  void setImageFileNameExtension(const std::string &imageExtension);

  void setFirstFrameIndex(long firstIndex);

  void open(ImageType &image);
  void close();

  void saveImage(ImageType &image);

private:
  /** Ultrasound image settings saved for writing at end of sequence*/
  ImageType m_frame;

  /** Sequence frameRate */
  double m_frameRate;

  /** First frame index*/
  long m_firstFrame;
  bool m_firstFrameIsSet;
  /** Count the frame number. Incremented directly after writing an image (contains next index) */
  long m_frameCount;

  /** file name of sequence settings file (ex : sequence.xml), images files names are deduced from it.*/
  std::string m_sequenceFileName;
  std::string m_genericImageFileName;
  std::string m_imagesExtension;
  bool m_fileNameIsSet;

  /** Top know if the sequence is already open*/
  bool is_open;
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Constructor.
*/
template<class ImageType>
usSequenceWriter<ImageType>::usSequenceWriter() : m_frame(), m_frameRate(0.0), m_firstFrame(0), m_firstFrameIsSet(false),
  m_frameCount(0), m_sequenceFileName(""),m_genericImageFileName(""),m_imagesExtension(".png"), m_fileNameIsSet(false), is_open(false)
{

}

/**
* Destructor.
*/
template<class ImageType>
usSequenceWriter<ImageType>::~usSequenceWriter()
{
  close();
}

/**
* FrameRate setter.
*/
template<class ImageType>
void usSequenceWriter<ImageType>::setFrameRate(double frameRate)
{
  m_frameRate = frameRate;
}

/**
* FrameRate getter.
*/
template<class ImageType>
double usSequenceWriter<ImageType>::getFrameRate() const
{
  return m_frameRate;
}

/**
* Settings fileName setter.
*/
template<class ImageType>
void usSequenceWriter<ImageType>::setSequenceFileName(const std::string &sequenceFileName)
{
  m_sequenceFileName = sequenceFileName;
  m_fileNameIsSet = true;
}

/**
* Image file extension setter. Set by default to ".png"
*/
template<class ImageType>
void usSequenceWriter<ImageType>::setImageFileNameExtension(const std::string &imageExtension)
{
  m_imagesExtension = imageExtension;
}

/**
* Sequence first index setter.
*/
template<class ImageType>
void usSequenceWriter<ImageType>::setFirstFrameIndex(long firstIndex)
{
  m_firstFrame = firstIndex;
  m_firstFrameIsSet = true;
}

/**
* Sequence opening.
*/
template<class ImageType>
void usSequenceWriter<ImageType>::open(ImageType &image)
{
  if(!m_fileNameIsSet)
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));

  //if not first index set, we begin at index 0
  if(!m_firstFrameIsSet) {
    m_firstFrame = 0;
    m_frameCount = 0;
    m_firstFrameIsSet = true;
  }

  //saving the settings for all the sequence
  m_frame = image;

  //Reading image
  char buffer[10];
  sprintf(buffer,"%ld",m_firstFrame);
  m_genericImageFileName = vpIoTools::splitChain(m_sequenceFileName,".")[0] + buffer + m_imagesExtension;
  std::cout << m_genericImageFileName << std::endl;
  vpImageIo::write(image,m_genericImageFileName);

  m_frameCount = m_firstFrame + 1;
  is_open = true;
}

/**
* Sequence closing for generic image type. NOT IMPLEMENTED !
*/
template<class ImageType>
void usSequenceWriter<ImageType>::close()
{
  throw(vpException(vpException::notImplementedError));
}

/**
* Sequence closing for usImageRF2D.
*/
template<>
void usSequenceWriter< usImageRF2D < unsigned char > >::close()
{
  if(!is_open)
    return;

  //saving settings
  usImageSettingsXmlParser xmlParser;
  xmlParser.setImagePreScanSettings(m_frame);
  xmlParser.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_RF);
  xmlParser.setSequenceFrameRate(m_frameRate);
  xmlParser.setSequenceStartNumber(m_firstFrame);
  xmlParser.setSequenceStopNumber(m_frameCount-1);
  xmlParser.setImageFileName(vpIoTools::splitChain(vpIoTools::getName(m_sequenceFileName),".")[0] + m_imagesExtension); //just image name without any directory name
  xmlParser.save(m_sequenceFileName);
}

/**
* Sequence closing for usImagePreScan2D.
*/
template<>
void usSequenceWriter < usImagePreScan2D < unsigned char > >::close()
{
  if(!is_open)
    return;

  //saving settings
  usImageSettingsXmlParser xmlParser;
  xmlParser.setImagePreScanSettings(m_frame);
  xmlParser.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_PRESCAN);
  xmlParser.setSequenceFrameRate(m_frameRate);
  xmlParser.setSequenceStartNumber(m_firstFrame);
  xmlParser.setSequenceStopNumber(m_frameCount-1);
  xmlParser.setImageFileName(vpIoTools::splitChain(vpIoTools::getName(m_sequenceFileName),".")[0] + m_imagesExtension); //just image name without any directory name
  xmlParser.save(m_sequenceFileName);
}

/**
* Sequence closing for usImagePostScan2D.
*/
template<>
void usSequenceWriter<usImagePostScan2D<unsigned char> >::close()
{
  if(!is_open)
    return;

  //saving settings
  usImageSettingsXmlParser xmlParser;
  xmlParser.setImagePostScanSettings(m_frame);
  xmlParser.setImageType(usImageSettingsXmlParser::IMAGE_TYPE_POSTSCAN);
  xmlParser.setSequenceFrameRate(m_frameRate);
  xmlParser.setSequenceStartNumber(m_firstFrame);
  xmlParser.setSequenceStopNumber(m_frameCount-1);
  xmlParser.setImageFileName(vpIoTools::splitChain(vpIoTools::getName(m_sequenceFileName),".")[0] + m_imagesExtension); //just image name without any directory name
  xmlParser.save(m_sequenceFileName);
}

/**
* Sequence last index setter.
*/
template<class ImageType>
void usSequenceWriter<ImageType>::saveImage(ImageType &image)
{
  if(!is_open)
    throw(vpException(vpException::fatalError,"trying to save an image without previously open the sequence"));

  //Writing image
  char buffer[10];
  sprintf(buffer,"%ld",m_frameCount);
  std::string imageFileName = vpIoTools::splitChain(m_sequenceFileName,".")[0] + buffer + m_imagesExtension;

  vpImageIo::write(image,imageFileName);

  m_frameCount++;
}

#endif //US_SEQUENCE_WRITER_H
