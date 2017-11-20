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
* @file usMHDSequenceWriter.h
* @brief Writer for a sequence of ultrasound images stored as mhd/raw files in a directory
*/

#ifndef US_MHD_SEQUENCE_WRITER_H
#define US_MHD_SEQUENCE_WRITER_H

#include <iostream>
#include <stdint.h>
#include <vector>

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usRawFileParser.h>

/**
 * @class usMHDSequenceWriter
 * @brief Writer for a sequence of images stored as mhd/raw files in a directory
 * Image filenames are set based on the following format: "image%05d.mhd" and "image%05d.raw".
 * An internal counter is incremented every time write() is called, to update the filename of the new image in the
 * sequence.
 * @ingroup module_ustk_io
 */
class VISP_EXPORT usMHDSequenceWriter
{
public:
  usMHDSequenceWriter();
  ~usMHDSequenceWriter();

  void setSequenceDirectory(const std::string sequenceDirectory);

  void write(const usImageRF2D<short int> &image, const uint64_t timestamp);
  void write(const usImagePreScan2D<unsigned char> &image, const uint64_t timestamp);
  void write(const usImagePostScan2D<unsigned char> &image, const uint64_t timestamp);
  void write(const usImageRF3D<short int> &image, const std::vector<uint64_t> timestamp);
  void write(const usImagePreScan3D<unsigned char> &image, const std::vector<uint64_t> timestamp);
  void write(const usImagePostScan3D<unsigned char> &image, const uint64_t timestamp);

private:
  std::string m_sequenceDirectory;

  us::ImageType m_sequenceImageType;

  int m_imageCounter;
};

#endif // US_MHD_SEQUENCE_WRITER_H
