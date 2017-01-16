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
 * @file us.h
 * @brief us namespace.
 */

#ifndef __us_h_
#define __us_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>
#include <visp3/ustk_core/usMotorSettings.h>
/**
 * @namespace us
 * @brief General tools
 * @ingroup module_ustk_core
 */
namespace us {

  /*! Enum to know the ultrasound image type
  Used in ultrasonix grabber to adapt the grabber when we receive the header, and in the mhd parser.*/
  typedef enum {
    UNKNOWN = -1, /*!< Unkownn format. */
    NOT_SET,      /*!< Not set (usefull for mhd parser). */
    RF_2D,        /*!< Case of 2D RF image. */
    RF_3D,        /*!< Case of 3D RF image. */
    PRESCAN_2D,   /*!< Case of 2D pre-scan image. */
    PRESCAN_3D,   /*!< Case of 3D pre-scan image. */
    POSTSCAN_2D,  /*!< Case of 2D post-scan image. */
    POSTSCAN_3D,  /*!< Case of 3D post-scan image. */
  }ImageType;

  VISP_EXPORT std::string getDataSetPath();
};


#endif // US_H

