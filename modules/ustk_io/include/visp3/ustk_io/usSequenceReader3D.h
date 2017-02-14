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
* @file usSequenceReader3D.h
* @brief Reading of sequences of ultrasound 3D images
*
* This class is used to read ultrasound images from a sequence of volumes.
*/

#ifndef __usSequenceReader3D_h_
#define __usSequenceReader3D_h_

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <visp3/ustk_io/usImageIo.h>

/**
* @class usSequenceReader3D
* @brief Reading of sequences of ultrasound images
* @ingroup module_ustk_io
*
* This class is used to grab ultrasound volumes from a sequence.
*
* @warning This class reads .vol files which don't contain transducer/motor informations. If yout want
* to use the volumes grabbed by this class make sure to complete them by yourself.
*/
template <class ImageType>
class usSequenceReader3D
{
private:

  void open(ImageType &image);

  /** Count the frame number when the class is used as a grabber (from 0 to volume number - 1).
   * Incremented directly after grabbing an image (contains next index) */
  long m_frameCount;

  /** file name of sequence settings file (ex : sequence.xml), images files names are deduced from information contained in it.*/
  std::string m_fileName;
  bool m_fileNameIsSet;

  /** Top know if the sequence is already open (m_volHeader have already been filled)*/
  bool is_open;

  /** Header info */
  usImageIo::VolHeader m_volHeader;

  /** file stream */
  std::ifstream m_volFile;

public:

  usSequenceReader3D();

  virtual ~usSequenceReader3D();

  //get images in grabber style
  void acquire(ImageType &image);

  //get image with its number in the sequence
  void getVolume(usImagePreScan3D<unsigned char> &preScanImage,int volumeNumberInSequence);

  /*!
    \return true if the end of the sequence is reached.
  */
  inline bool end() {
    if(!is_open) {
      ImageType image;
      open(image);
      m_frameCount--;
    }
    if (m_frameCount > m_volHeader.volumes)
      return true;
    return false;
  }

  long getImageNumber() const {return m_frameCount;}

  void setSequenceFileName(const std::string &sequenceFileName);
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Default constructor.
*/
template<class ImageType>
usSequenceReader3D<ImageType>::usSequenceReader3D() : m_frameCount(0),
 m_fileName(""), m_fileNameIsSet(false), is_open(false), m_volHeader()
{

}

/**
* Destructor.
*/
template<class ImageType>
usSequenceReader3D<ImageType>::~usSequenceReader3D()
{

}

/**
* FileName setter.
* @param fileName Name of the .vol file.
*/
template<class ImageType>
void usSequenceReader3D<ImageType>::setSequenceFileName(const std::string &fileName)
{
  m_fileName = fileName;
  m_fileNameIsSet = true;
}

/**
* Sequence opening.
* @param image First image of the sequence to read.
*/
template<class ImageType>
void usSequenceReader3D<ImageType>::open(ImageType &image)
{
  (void) image;
  throw(vpException(vpException::notImplementedError));
}
/*
template<>
void usSequenceReader3D<usImageRF2D<unsigned char> >::open(usImageRF2D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }
  usImageSettingsXmlParser xmlParser;
  xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(xmlParser.getSequenceStartNumber());
  setLastFrameIndex(xmlParser.getSequenceStopNumber());
  m_genericImageFileName = xmlParser.getImageFileName();

  //saving the settings for all the rf sequence
  m_frame.setTransducerRadius(xmlParser.getTransducerSettings().getTransducerRadius());
  m_frame.setScanLinePitch(xmlParser.getTransducerSettings().getScanLinePitch());
  m_frame.setScanLineNumber(xmlParser.getTransducerSettings().getScanLineNumber());
  m_frame.setTransducerConvexity(xmlParser.getTransducerSettings().isTransducerConvex());
  m_frame.setAxialResolution(xmlParser.getAxialResolution());

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
}*/

template<>
void usSequenceReader3D<usImagePreScan3D<unsigned char> >::open(usImagePreScan3D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence file name not set"));
  }

  if(usImageIo::getHeaderFormat(m_fileName) != usImageIo::FORMAT_VOL)
    throw(vpException(vpException::ioError, "only .vol files are supported for 3D sequence reading"));

  //INIT
  int szHeader = sizeof(usImageIo::VolHeader);
  int n = 0;
  char byte;

  //FILE OPENING
  m_volFile.open( m_fileName.c_str(), std::ios::in|std::ios::binary);

  //READING HEADER
  while (n < szHeader) {
    m_volFile.read(&byte, 1);
    ((char*)&m_volHeader)[n] = byte;
    n++;
  }

  //CHECK IMAGE TYPE
  if (m_volHeader.type != 0)
    throw(vpException(vpException::badValue, "trying to read non-prescan in .vol file"));

  //CHECK DATA TYPE
  if (m_volHeader.ss != 8)
    throw(vpException(vpException::badValue, ".vol file doesn't contain unsigned char data"));

  //READING DATA
  image.resize(m_volHeader.w, m_volHeader.h, m_volHeader.fpv);
  n = 0;
  unsigned char voxel;
  for (int k = 0; k < m_volHeader.fpv; k++) {
    for (int j = 0; j < m_volHeader.w; j++) {
      for (int i = 0; i < m_volHeader.h; i++) {
        m_volFile.read((char*)&voxel, 1);
        image(j, i, k, voxel);
      }
    }
  }

  m_frameCount = 1;
  is_open = true;
}
/*
template<>
void usSequenceReader3D<usImagePostScan2D<unsigned char> >::open(usImagePostScan2D<unsigned char> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }

  usImageSettingsXmlParser xmlParser;
  xmlParser.parse(m_sequenceFileName);

  setFirstFrameIndex(xmlParser.getSequenceStartNumber());
  setLastFrameIndex(xmlParser.getSequenceStopNumber());
  m_genericImageFileName = xmlParser.getImageFileName();

  //saving the settings for all the post scan sequence
  m_frame.setTransducerRadius(xmlParser.getTransducerSettings().getTransducerRadius());
  m_frame.setScanLinePitch(xmlParser.getTransducerSettings().getScanLinePitch());
  m_frame.setScanLineNumber(xmlParser.getTransducerSettings().getScanLineNumber());
  m_frame.setTransducerConvexity(xmlParser.getTransducerSettings().isTransducerConvex());
  m_frame.setWidthResolution(xmlParser.getWidthResolution());
  m_frame.setHeightResolution(xmlParser.getHeightResolution());

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
*/
/**
* Sequence image acquisition (grabber-style : an internal counter is incremented to open next image at the next call).
* @param image Image of the sequence to read.
*/
template<>
void usSequenceReader3D<usImagePreScan3D<unsigned char> >::acquire(usImagePreScan3D<unsigned char> &image)
{
  if (!is_open) {
    this->open(image);
    return;
  }
  //READING DATA
  image.resize(m_volHeader.w, m_volHeader.h, m_volHeader.fpv);
  unsigned char voxel;
  for (int k = 0; k < m_volHeader.fpv; k++) {
    for (int j = 0; j < m_volHeader.w; j++) {
      for (int i = 0; i < m_volHeader.h; i++) {
        m_volFile.read((char*)&voxel, 1);
        image(j, i, k, voxel);
      }
    }
  }

  m_frameCount ++;
}

/**
* Read a 3D unsigned char pre-scan ultrasound image contained in a sequence. Works only for .vol files.
* @param [out] preScanImage The pre-scan image to read.
* @param [in] volumeNumberInSequence The image number of the image to read in the sequence (from 0 to total volume Number - 1).
*
* @warning Using this method between 2 calls of acquire() breaks the sequence grabbing system (the offset in the vol file is broken by calling this method).
*/
template<>
void usSequenceReader3D<usImagePreScan3D<unsigned char> >::getVolume(usImagePreScan3D<unsigned char> &preScanImage,int volumeNumberInSequence)
{
  if (!is_open) {
    this->open(preScanImage);
    return;
  }

    //CHECK IMAGE TYPE
    if (m_volHeader.type != 0)
      throw(vpException(vpException::badValue, "trying to read non-prescan in .vol file"));

    //CHECK DATA TYPE
    if (m_volHeader.ss != 8)
      throw(vpException(vpException::badValue, ".vol file doesn't contain unsigned char data"));

    // OFFSET
    // to select volume to read (in .vol files multiple volumes can be stacked after the header part)
    int szHeader = sizeof(usImageIo::VolHeader);
    int offset = szHeader + volumeNumberInSequence * m_volHeader.fpv * m_volHeader.w * m_volHeader.h;
    m_volFile.seekg(offset);

    //READING DATA
    preScanImage.resize(m_volHeader.w, m_volHeader.h, m_volHeader.fpv);
    unsigned char voxel;
    for (int k = 0; k < m_volHeader.fpv; k++) {
      for (int j = 0; j < m_volHeader.w; j++) {
        for (int i = 0; i < m_volHeader.h; i++) {
          m_volFile.read((char*)&voxel, 1);
          preScanImage(j, i, k, voxel);
        }
      }
    }
}

#endif // US_SEQUENCE_READER_3D_H
