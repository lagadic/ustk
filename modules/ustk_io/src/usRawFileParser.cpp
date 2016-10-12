/****************************************************************************
*
* This file is part of the UsTk software.
* Copyright(C) 2014 by Inria.All rights reserved.
*
* This program is free software : you can redistribute it and / or modify
* it under the terms of the GNU General Public License("GPL") as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
* See the file COPYING at the root directory of this source
* distribution for additional information about the GNU GPL.
*
* This software was developed at :
*INRIA Rennes - Bretagne Atlantique
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* France
* http ://www.irisa.fr/lagadic
  *
  * If you have questions regarding the use of this file, please contact the
  * authors at Alexandre.Krupa@inria.fr
  *
  * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
  * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
  *
  *
  * Authors:
*Marc Pouliquen
*
***************************************************************************** /

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
* @param image3D 3D-image to fill.
* @param rawFilename File name of the image to read (with .raw extension).
*/
void usRawFileParser::read(usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream >> image3D[i];
    i++;
  }
  fileStream.close();
}

/**
* Writing method for 3D images.
* @param image3D 3D-image to write.
* @param rawFilename File name of the image to write (with .raw extension).
*/
void usRawFileParser::write(const usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream << image3D[i];
    i++;
  }
  fileStream.close();
}

/**
* Reading method for 2D images.
* @param image2D 2D-image to fill.
* @param rawFilename File name of the image to read (with .raw extension).
*/
void usRawFileParser::read(vpImage<unsigned char> &image2D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  for (unsigned int i = 0; i < image2D.getWidth(); i++) {
    for (unsigned int j = 0; j < image2D.getHeight(); j++) {
      fileStream >> image2D[j][i];
    }
  }
  fileStream.close();
}

/**
* Writing method for 2D images.
* @param image2D 2D-image to write.
* @param rawFilename File name of the image to write (with .raw extension).
*/
void usRawFileParser::write(const vpImage<unsigned char> &image2D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  for (unsigned int i = 0; i < image2D.getWidth(); i++) {
    for (unsigned int j = 0; j < image2D.getHeight(); j++) {
      fileStream << image2D[j][i];
    }
  }
  fileStream.close();
}
