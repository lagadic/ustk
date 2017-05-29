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
* @file usDataFrameReader.h
* @brief Reading rf data.
*
* This class is used to read ultrasound signal as rf data.
*/

#ifndef __usDataFrameReader_h_
#define __usDataFrameReader_h_

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

#include <visp3/ustk_io/usImageIo.h>

/**
* @class usDataFrameReader
* @brief Reading of sequences of ultrasound images
* @ingroup module_ustk_io
*
* This class is used to read ultrasound signal as rf data.
*
* @warning This class reads .rf files which don't contain transducer/motor informations. If yout want
* to use the data grabbed by this class make sure to complete them by yourself.
*/
template <class ImageType>
class usDataFrameReader
{
private:
  /** data file name (ex : signal.rf).*/
  std::string m_fileName;
  bool m_fileNameIsSet;

  /** Header info */
  usImageIo::frameDataHeader m_header;

  /** file stream */
  std::ifstream m_dataFile;

public:

  usDataFrameReader();

  virtual ~usDataFrameReader();

  void open(ImageType &image);

  void setFileName(const std::string &sequenceFileName);
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Default constructor.
*/
template<class ImageType>
usDataFrameReader<ImageType>::usDataFrameReader() : m_fileName(""), m_fileNameIsSet(false), m_header()
{

}

/**
* Destructor.
*/
template<class ImageType>
usDataFrameReader<ImageType>::~usDataFrameReader()
{

}

/**
* FileName setter.
* @param fileName Name of the .vol file.
*/
template<class ImageType>
void usDataFrameReader<ImageType>::setFileName(const std::string &fileName)
{
  m_fileName = fileName;
  m_fileNameIsSet = true;
}

/**
* Sequence opening.
* @param image First image of the sequence to read.
*/
template<class ImageType>
void usDataFrameReader<ImageType>::open(ImageType &image)
{
  (void) image;
  throw(vpException(vpException::notImplementedError));
}

template<>
void usDataFrameReader<usImageRF2D<short int> >::open(usImageRF2D<short int> &image)
{
  if(!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }

  if(usImageIo::getHeaderFormat(m_fileName) != usImageIo::FORMAT_RF)
    throw(vpException(vpException::ioError, "only .rf files are supported"));

  //INIT
  int szHeader = sizeof(usImageIo::frameDataHeader);
  int n = 0;
  char byte;

  //FILE OPENING
  m_dataFile.open( m_fileName.c_str(), std::ios::in|std::ios::binary);

  //READING HEADER
  while (n < szHeader) {
    m_dataFile.read(&byte, 1);
    ((char*)&m_header)[n] = byte;
    n++;
  }

  //CHECK IMAGE TYPE
  if (m_header.type != 16)
    throw(vpException(vpException::badValue, "trying to read non-rf data in .rf file"));

  //CHECK DATA TYPE
  if (m_header.ss != 16)
    throw(vpException(vpException::badValue, ".vol file doesn't contain short data"));

  image.resize( m_header.h, m_header.w);
  short sample;
  for (int i = 0; i < m_header.h; i++) {
    for (int j = 0; j < m_header.w; j++) {
      m_dataFile.read((char *)&sample, sizeof(short));
      image(i, j, sample);
    }
  }
  m_dataFile.close();
}

#endif // US_SEQUENCE_READER_3D_H
