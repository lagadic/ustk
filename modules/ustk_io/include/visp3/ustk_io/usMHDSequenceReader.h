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
* @file usMHDSequenceReader.h
* @brief Reader for a sequence of ultrasound images stored as mhd/raw files in a directory
*/

#ifndef US_MHD_SEQUENCE_READER_H
#define US_MHD_SEQUENCE_READER_H

#include <vector>
#include <iostream>
#include <stdint.h>

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usRawFileParser.h>

/**
 * @class usMHDSequenceReader
 * @brief Reader for a sequence of images stored as mhd/raw files in a directory
 * Image sequence files order have to respect alphabetical order in the directory, and be the same for corresponding mhd and raw files.
 * For example : image1.mhd, image1.raw, image2.mhd, image2.raw, ...
 * The directory must contain exactly the same number of images as the number of images contained in the sequence (1 mhd file and 1 raw file per image of the sequence).
 * @ingroup module_ustk_io
 */
class usMHDSequenceReader
{
public:

  usMHDSequenceReader();
  ~usMHDSequenceReader();

  void acquire(usImageRF2D<short int> & image, uint64_t & timestamp);
  void acquire(usImageRF3D<short int> & image, uint64_t & timestamp);
  void acquire(usImagePreScan3D<unsigned char> & image, uint64_t & timestamp);
  void acquire(usImagePostScan3D<unsigned char> & image, uint64_t & timestamp);

  void setSequenceDirectory(const std::string sequenceDirectory);

private:

  std::string m_sequenceDirectory;

  us::ImageType m_sequenceImageType;

  std::vector<std::string> m_sequenceFiles;

  int m_totalImageNumber;
  int m_imageCounter;
};

#endif // US_MHD_SEQUENCE_READER_H
