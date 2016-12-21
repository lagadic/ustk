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
* @file usMeterPixelConversion.cpp
* @brief Conversion from a position in meters in the space to the equivalent pixel position in the ultrasound image.
*/

#include <visp3/ustk_core/usMeterPixelConversion.h>

/**
* Conversion method for 2D ultrasound images.
*
* @param [in] image 2D Post-scan image with transducer settings well filled.
* @param [in] x Position in meters along x axis to convert.
* @param [in] y Position in meters along y axis to convert.
* @param [out] u Converted position in pixels along x axis.
* @param [out] v Converted position in pixels along y axis.
*/
void usMeterPixelConversion::convert(const usImagePostScan2D<unsigned char> &image,
                                     const double &x, const double &y,
                                     double &u,  double &v)
{
  //checking transducer settings to apply corresponding transformation
  //First convex probe type
  if(image.isTransducerConvex()) {
    u = (x + ((image.getWidth() * image.getWidthResolution()) / 2)) / image.getWidthResolution();
    v = (y - image.getTransducerRadius() * std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch()/2)) / image.getHeightResolution();
  }
  //Then linear probe type
  else {
    u = (x + ((image.getWidth() * image.getWidthResolution()) / 2)) / image.getWidthResolution();
    v = y / image.getHeightResolution();
  }
}

/**
* Conversion method for 3D ultrasound images.
*
* @param [in] image 2D Post-scan image with voxels spacings, transducer settings and motor settings well filled.
* @param [in] x Position in meters along x axis to convert.
* @param [in] y Position in meters along y axis to convert.
* @param [in] z Position in meters along z axis to convert.
* @param [out] u Converted position in pixels along x axis.
* @param [out] v Converted position in pixels along y axis.
* @param [out] w Converted position in pixels along z axis.
*/
void usMeterPixelConversion::convert(const usImagePostScan3D<unsigned char> &image,
                                     const double &x, const double &y, const double &z,
                                     double &u,  double &v, double &w)
{
  //checking transducer/motor settings to apply corresponding transformation
    if(!image.isTransducerConvex()) { //linear transducer
      if(image.getMotorType() == usMotorSettings::LinearMotor) { //linear motor
        u = (x + ((image.getDimX() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
        v = y / image.getElementSpacingY();
        w = (z + image.getDimZ() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
      }
      else if (image.getMotorType() == usMotorSettings::TiltingMotor) { //tilting motor
        u = (x + ((image.getDimX() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
        v = (y - image.getMotorRadius() * std::cos((image.getFrameNumber() - 1) * image.getFramePitch() / 2) + image.getMotorRadius() )/ image.getElementSpacingY();
        w = (z + image.getDimZ() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
      }
      else
        throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
  //Then convex transducer
  else {
    if(image.getMotorType() == usMotorSettings::LinearMotor) { //linear motor
      u = (x + ((image.getDimX() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
      v = (y - image.getTransducerRadius() * std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch() / 2)) / image.getElementSpacingY();
      w = (z + image.getDimZ() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    }
    else if (image.getMotorType() == usMotorSettings::TiltingMotor) { //tilting motor
      u = (x + ((image.getDimX() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();

      v = (y -
      (image.getTransducerRadius() * //Rmin
      std::cos((image.getScanLineNumber()-1)*image.getScanLinePitch()/2) //cos (FOV/2)
      - (image.getTransducerRadius() - image.getMotorRadius()))//Delta R
      * std::cos((image.getFrameNumber()-1) * image.getFramePitch() / 2) //cos(Phi/2)
      - (image.getTransducerRadius() - image.getMotorRadius()))//Delta R
      / image.getElementSpacingY();


      w = (z + image.getDimZ() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    }
    else
      throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
}
