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
* @file usRawFileParser.h
* @brief Input/output operations between ultrasound images and the assiciated raw files.
*/

#ifndef __usRawFileParser_h_
#define __usRawFileParser_h_

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
  void read(vpImage<short> &image2D, const std::string &mhdFileName);
  void write(const vpImage<short> &image2D, const std::string &rawFileName);
  //@}

  /** @name 3D io */
  //@{
  void read(usImage3D<unsigned char> &image3D, const std::string &mhdFileName);
  void write(const usImage3D<unsigned char> &image3D, const std::string &rawFileName);
  void read(usImage3D<short> &image3D, const std::string &mhdFileName);
  void write(const usImage3D<short> &image3D, const std::string &rawFileName);
  //@}
};
#endif //US_RAW_FILE_PARSER_H
#endif //DOXYGEN_SHOULD_SKIP_THIS
