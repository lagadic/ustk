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

#include <visp3/ustk_core/us.h>

/**
* Method to get directly the pre-scan settings of the ultrasonix 4DC7 probe.
* The scanline number is not set here because for pre-scan image it is automatically set by the image width for pre-scan images.
* @param [out] preScanSettings Pre-scan settings of the 4DC7 probe.
*/
void us::getUltrasonix4DC7PreScanSettings(usImagePreScanSettings &preScanSettings){
   preScanSettings.setTransducerConvexity(true);
   preScanSettings.setTransducerRadius(0.04);
   preScanSettings.setAxialResolution(0.000308);
   preScanSettings.setScanLinePitch(0.010625);
}

/**
* Method to get directly the motor settings of the ultrasonix 4DC7 probe.
* The frame number is not set here because for pre-scan image it is automatically set by the image dimention in Z axis for 3D pre-scan images.
* @param [out] motorSettings Motor settings of the 4DC7 probe.
*/
void us::getUltrasonix4DC7MotorSettings(usMotorSettings &motorSettings){
  motorSettings.setFramePitch(0.0255342);
  motorSettings.setMotorRadius(0.02725);
  motorSettings.setMotorType(usMotorSettings::TiltingMotor);
}
