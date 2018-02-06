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
void usMeterPixelConversion::convert(const usImagePostScan2D<unsigned char> &image, const double &x, const double &y,
                                     double &u, double &v)
{
  // checking transducer settings to apply corresponding transformation
  // First convex probe type
  if (image.isTransducerConvex()) {
    u = (x + ((image.getWidth() * image.getWidthResolution()) / 2)) / image.getWidthResolution();
    v = (y - image.getTransducerRadius() * std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch() / 2)) /
        image.getHeightResolution();
  }
  // Then linear probe type
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
void usMeterPixelConversion::convert(const usImagePostScan3D<unsigned char> &image, const double &x, const double &y,
                                     const double &z, double &u, double &v, double &w)
{
  // checking transducer/motor settings to apply corresponding transformation
  if (!image.isTransducerConvex()) {                            // linear transducer
    if (image.getMotorType() == usMotorSettings::LinearMotor) { // linear motor
      u = (x + ((image.getWidth() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
      v = y / image.getElementSpacingY();
      w = (z + image.getNumberOfFrames() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    } else if (image.getMotorType() == usMotorSettings::TiltingMotor) { // tilting motor
      u = (x + ((image.getWidth() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
      v = (y - image.getMotorRadius() * std::cos((image.getFrameNumber() - 1) * image.getFramePitch() / 2) +
           image.getMotorRadius()) /
          image.getElementSpacingY();
      w = (z + image.getNumberOfFrames() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    } else
      throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
  // Then convex transducer
  else {
    if (image.getMotorType() == usMotorSettings::LinearMotor) { // linear motor
      u = (x + ((image.getWidth() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();
      v = (y - image.getTransducerRadius() * std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch() / 2)) /
          image.getElementSpacingY();
      w = (z + image.getNumberOfFrames() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    } else if (image.getMotorType() == usMotorSettings::TiltingMotor) { // tilting motor
      u = (x + ((image.getWidth() * image.getElementSpacingX()) / 2)) / image.getElementSpacingX();

      v = (y -
           (image.getTransducerRadius() *                                                // Rmin
                std::cos((image.getScanLineNumber() - 1) * image.getScanLinePitch() / 2) // cos (FOV/2)
            - (image.getTransducerRadius() - image.getMotorRadius()))                    // Delta R
               * std::cos((image.getFrameNumber() - 1) * image.getFramePitch() / 2)      // cos(Phi/2)
           - (image.getTransducerRadius() - image.getMotorRadius()))                     // Delta R
          / image.getElementSpacingY();

      w = (z + image.getNumberOfFrames() * image.getElementSpacingZ() / 2) / image.getElementSpacingZ();
    } else
      throw(vpException(vpException::notImplementedError, "Rotationnal Motor is not available yet."));
  }
}
