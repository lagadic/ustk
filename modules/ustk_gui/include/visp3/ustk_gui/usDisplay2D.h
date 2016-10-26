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
 * @file usDisplay2D.h
 * @brief Class to display a ultrasound image at screen, and interact with it.
 */

#ifndef US_DISPLAY_2D_H
#define US_DISPLAY_2D_H

#include <string>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>


/**
 * @class usDisplay2D
 * @brief Class to display a ultrasound image at screen, and interact with it.
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usDisplay2D
{
public:
  usDisplay2D();
  virtual ~usDisplay2D();
};

#endif //US_DISPLAY_2D_H
