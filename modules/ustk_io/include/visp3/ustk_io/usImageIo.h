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
 * @file usImageIo.h
 * @brief Input/output operations between ultrasound data and image files and their associated header
 * (containing usTransducerSettings, usMotorSettings, and pre-scan or post-scan info).
 */

#ifndef __usImageIo_h_
#define __usImageIo_h_

#include <string>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_io/usMetaHeaderParser.h>
#include <visp3/ustk_io/usImageSettingsXmlParser.h>


/**
 * @class usImageIo
 * @brief Input/output operations between ultrasound data and files (header + image file).
 * @ingroup module_ustk_io
 *
 */
class VISP_EXPORT usImageIo
{
private:

  static std::string getExtension(const std::string &filename);

public:

  /** @name RF io */
  //@{
  static void read(usImageRF2D<unsigned char> &rfImage,const std::string &headerFileName);
  static void read(usImageRF3D<unsigned char> &rfImage3D, const std::string &headerFileName);

  static void write(const usImageRF2D<unsigned char> &rfImage, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImageRF2D<unsigned char> &rfImage, const std::string &headerFileName);
  static void write(const usImageRF3D<unsigned char> &rfImage3D, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImageRF3D<unsigned char> &rfImage3D, const std::string &headerFileName);

  //@}

  /** @name Pre-scan io */
  //@{
  static void read(usImagePreScan2D<unsigned char> &preScanImage,const std::string &headerFileName);
  static void read(usImagePreScan3D<unsigned char> &preScanImage3D, const std::string &headerFileName);

  //Doubles types writing not implemented
  static void read(usImagePreScan2D<double> &preScanImage,const std::string &headerFileName);
  static void read(usImagePreScan3D<double> &preScanImage3D,const std::string &headerFileName);

  static void write(const usImagePreScan2D<unsigned char> &preScanImage, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePreScan2D<unsigned char> &preScanImage, const std::string &headerFileName);

  static void write(const usImagePreScan3D<unsigned char> &preScanImage3D, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePreScan3D<unsigned char> &preScanImage3D, const std::string &headerFileName);

  //Doubles types writing not implemented
  static void write(const usImagePreScan2D<double> &preScanImage, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePreScan2D<double> &preScanImage, const std::string &headerFileName);
  static void write(const usImagePreScan3D<double> &preScanImage3D, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePreScan3D<double> &preScanImage3D, const std::string &headerFileName);
  //@}

  /** @name Post-scan io */
  //@{
  static void read(usImagePostScan2D<unsigned char> &postScanImage, const std::string &headerFileName);
  static void read(usImagePostScan3D<unsigned char> &postScanImage3D, const std::string &headerFileName);

  static void write(const usImagePostScan2D<unsigned char> &postScanImage, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePostScan2D<unsigned char> &postScanImage, const std::string &headerFileName);
  static void write(const usImagePostScan3D<unsigned char> &postScanImage3D, const std::string &headerFileName,
                    const std::string &imageExtension2D);
  static void write(const usImagePostScan3D<unsigned char> &postScanImage3D, const std::string &headerFileName);
  //@}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct VolHeader
  {
    int type;     // Data type(0: prescan B, 1 : postscan B, 2 : rf, 3 : vel / var, 5 = B + flow RGB)
    int volumes;  // Number of volumes in the file
    int fpv;      // Number of frames per volumes
    int w;        // frame width
    int h;        // frame height
    int ss;       // sample size in bits
    int degPerFr; // degree step between frames
  };
#endif //DOXYGEN_SHOULD_SKIP_THIS

  typedef enum
  {
    FORMAT_XML,
    FORMAT_MHD,
    FORMAT_VOL,
    HEADER_FORMAT_UNKNOWN
  } usHeaderFormatType;

  static usHeaderFormatType getHeaderFormat(const std::string &headerfilename);
};

#endif //US_IMAGE_IO_H
