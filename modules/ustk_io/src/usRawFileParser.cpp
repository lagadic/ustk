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
* @file usRawFileParser.cpp
* @brief Input/output operations between ultrasound images and binary raw files.
*/

#include <visp3/core/vpException.h>
#include <visp3/ustk_io/usRawFileParser.h>
#include <fstream>
#include <iostream>

/**
* Reading method for 3D images.
* @param[out] image3D 3D-image to fill.
* @param[in] rawFilename File name of the image to read (with .raw extension).
*/
void usRawFileParser::read(usImage3D<unsigned char> &image3D, const std::string &rawFilename)
{
  std::ifstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    char c;
    fileStream.get(c);
    image3D[i] = c;
    i++;
  }
  fileStream.close();
}

/**
* Writing method for 3D images.
* @param image3D 3D-image to write.
* @param rawFilename File name of the image to write (with .raw extension).
*/
void usRawFileParser::write(const usImage3D<unsigned char> &image3D, const std::string &rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream.put(image3D[i]);
    i++;
  }
  fileStream.close();
}

/**
* Reading method for 2D images.
* @param[out] image2D 2D-image to fill.
* @param[in] rawFilename File name of the image to read (with .raw extension).
*/
void usRawFileParser::read(vpImage<unsigned char> &image2D, const std::string &rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  fileStream.read((char *)image2D.bitmap, image2D.getSize());
  fileStream.close();
}

/**
* Writing method for 2D images.
* @param image2D 2D-image to write.
* @param rawFilename File name of the image to write (with .raw extension).
*/
void usRawFileParser::write(const vpImage<unsigned char> &image2D, const std::string &rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  fileStream.write((char *)image2D.bitmap, image2D.getSize());
  fileStream.close();
}
#endif //DOXYGEN_SHOULD_SKIP_THIS
