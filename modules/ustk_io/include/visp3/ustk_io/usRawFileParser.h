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
* @file usRawFileParser.h
* @brief Input/output operations between ultrasound images and the assiciated raw files.
*/

#ifndef US_RAW_FILE_PARSER_H
#define US_RAW_FILE_PARSER_H

#include<string>
#include<map>
#include <cstdlib>
#include <string>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/ustk_core/usImage3D.h>

/**
 * @class usRawFileParser
 * @brief Raw data parser.
 * @ingroup module_ustk_io
 */
class VISP_EXPORT usRawFileParser {

public:

  /** @name 2D io */
  //@{
  void read(vpImage<unsigned char> &image2D, const std::string &mhdFileName);
  void write(const vpImage<unsigned char> &image2D, const std::string &rawFileName);
  //@}

  /** @name 3D io */
  //@{
  void read(usImage3D<unsigned char> &image3D, const std::string &mhdFileName);
  void write(const usImage3D<unsigned char> &image3D, const std::string &rawFileName);
  //@}
};
#endif //US_RAW_FILE_PARSER_H
