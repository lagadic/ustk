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
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usGrabberSonosite.h
 * @brief Grabbing frames from sonosite probe, based on vpV4l2Grabber.
 * @author Marc Pouliquen
 */

#ifndef US_BACK_SCAN_CONVERTER_2D_H
#define US_BACK_SCAN_CONVERTER_2D_H

#include <visp3/sensor/vpV4l2Grabber.h>

/**
 * @class usGrabberSonosite
 * @brief Grabbing frames from sonosite probe, based on vpV4l2Grabber.
 * @author Marc Pouliquen
 */
class VISP_EXPORT usGrabberSonosite : public vpV4l2Grabber
{
 public:

  usBackScanConverter2D();

  ~usBackScanConverter2D();
};

#endif // US_BACK_SCAN_CONVERTER_2D_H
