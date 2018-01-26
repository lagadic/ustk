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

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
* @file usRfReader.cpp
* @brief Reading rf data.
*/

#include <visp3/ustk_io/usRfReader.h>

/**
* Default constructor.
*/
usRfReader::usRfReader() : m_fileName(""), m_fileNameIsSet(false), m_header() {}

/**
* Destructor.
*/
usRfReader::~usRfReader() {}

/**
* FileName setter.
* @param fileName Name of the .vol file.
*/
void usRfReader::setFileName(const std::string &fileName)
{
  m_fileName = fileName;
  m_fileNameIsSet = true;
}

/**
* Sequence opening.
* @param image First image of the sequence to read.
*/
void usRfReader::open(usImageRF2D<short int> &image)
{
  if (!m_fileNameIsSet) {
    throw(vpException(vpException::badValue, "Sequence settings file name not set"));
  }

  if (usImageIo::getHeaderFormat(m_fileName) != usImageIo::FORMAT_RF)
    throw(vpException(vpException::ioError, "only .rf files are supported"));

  // INIT
  int szHeader = sizeof(usImageIo::FrameHeader);
  int n = 0;
  char byte;

  // FILE OPENING
  m_dataFile.open(m_fileName.c_str(), std::ios::in | std::ios::binary);

  // READING HEADER
  while (n < szHeader) {
    m_dataFile.read(&byte, 1);
    ((char *)&m_header)[n] = byte;
    n++;
  }

  // CHECK IMAGE TYPE
  if (m_header.type != 16)
    throw(vpException(vpException::badValue, "trying to read non-rf data in .rf file"));

  // CHECK DATA TYPE
  if (m_header.ss != 16)
    throw(vpException(vpException::badValue, ".vol file doesn't contain short data"));

  image.resize(m_header.h, m_header.w);
  short sample;
  for (int i = 0; i < m_header.h; i++) {
    for (int j = 0; j < m_header.w; j++) {
      m_dataFile.read((char *)&sample, sizeof(short));
      image[j][i] = sample;
    }
  }
  m_dataFile.close();
}
#endif // DOXYGEN_SHOULD_SKIP_THIS
