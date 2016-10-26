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
* @file usPixelMeterConversion.cpp
* @brief Conversion between a position in meters in the space and the equivalent pixel position in the ultrasound image.
*/

#include <algorithm>
#include <visp3/ustk_core/usPixelMeterConversion.h>


void usPixelMeterConversion::convert(const usImagePostScan2D<unsigned char> &image,
                                     const double &u, const double &v,
                                     double &x,  double &y)
{
  //checking transducer settings to apply corresponding transformation
  //First convex probe type
  if(image.isTransducerConvex()) {
    x = image.getWidthResolution() * u - ((image.getWidth()*image.getWidthResolution())/2);
    y = image.getHeightResolution() * v + image.getTransducerRadius() * std::cos((image.getScanLineNumber()-1)*image.getScanLinePitch()/2);
  }
  //Then linear probe type
  else {
    x = image.getWidthResolution() * u - ((image.getWidth()*image.getWidthResolution()-1)/2);
    y = image.getHeightResolution() * v;
  }
}

/**
 * @brief Conversion for 3D post-scan images.
 * @param image
 * @param u
 * @param v
 * @param w
 * @param x
 * @param y
 * @param z
 *
 * @warning Make sure you completed the following transducer settings, the motor settings, and the image settings (element spacing) before the conversion.
 */
void usPixelMeterConversion::convert(const usImagePostScan3D<unsigned char> &image,
                                     const double &u, const double &v, const double &w,
                                     double &x,  double &y, double &z)
{
  //checking transducer/motor settings to apply corresponding transformation
    if(!image.isTransducerConvex()) { //linear transducer
      if(image.getMotorType() == usMotorSettings::LinearMotor) { //linear motor
        x = image.getElementSpacingX() * u - ((image.getDimX()*image.getElementSpacingX())/2);
        y = image.getElementSpacingY() * v;
        z = image.getElementSpacingZ() * w - (image.getDimZ()*image.getElementSpacingZ()/2);
      }
      else if (image.getMotorType() == usMotorSettings::TiltingMotor) { //tilting motor
        x = image.getElementSpacingX() * u - ((image.getDimX()*image.getElementSpacingX())/2);
        y = image.getElementSpacingY() * v + image.getMotorRadius() * std::cos((image.getFrameNumber() - 1) * image.getFramePitch() / 2) - image.getMotorRadius();
        z = image.getElementSpacingZ() * w - (image.getDimZ()*image.getElementSpacingZ()/2);
      }
      else
        throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
  //Then convex transducer
  else {
    if(image.getMotorType() == usMotorSettings::LinearMotor) { //linear motor
      x = image.getElementSpacingX() * u - (image.getDimX() * image.getElementSpacingX() / 2);
      y = image.getElementSpacingY() * v + image.getTransducerRadius() * std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch() / 2);
      z = image.getElementSpacingZ() * w - (image.getDimZ() * image.getElementSpacingZ() / 2);
    }
    else if (image.getMotorType() == usMotorSettings::TiltingMotor) { //tilting motor
      x = image.getElementSpacingX() * u - (image.getDimX() * image.getElementSpacingX() / 2);
      //TO CORRECT
      //y = image.getElementSpacingY() * v + image.getTransducerRadius() * std::cos((image.getScanLineNumber()-1)*image.getScanLinePitch()/2);
      z = image.getElementSpacingZ() * w - (image.getDimZ() * image.getElementSpacingZ() / 2);

    }
    else
      throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
}
