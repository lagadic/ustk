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
 * Marc Pouliquen
 *
 *****************************************************************************/

#include<visp3/ustk_core/usImagePostScanSettings.h>

 /**
 * Default constructor, all parameters set de default values.
 */
usImagePostScanSettings::usImagePostScanSettings()
: usTransducerSettings(), m_widthResolution(0.0), m_heightResolution(0.0)
{

}

/**
* Copy constructor.
* @param other usImagePostScanSettings to copy.
*/
usImagePostScanSettings::usImagePostScanSettings(const usImagePostScanSettings &other)
: usTransducerSettings(other), m_widthResolution(other.getHeightResolution()), m_heightResolution(other.getWidthResolution())
{

}

/**
* Full parameters constructor, all parameters settables.
* @param probeRadius Radius of the probe used to acquire the image.
* @param scanLinePitch Pitch between 2 scanlines (in radians if isTransducerConvex = true, in meters otherwise).
* @param isTransducerConvex True if the transducer is convex, false if it is linear.
* @param heightResolution Post-scan image height resolution in meters (distance between two pixels).
* @param widthResolution Post-scan image width resolution in meters (distance between two pixels).
*/
usImagePostScanSettings::usImagePostScanSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex,
 double heightResolution, double widthResolution)
: usTransducerSettings(probeRadius, scanLinePitch, isTransducerConvex),
  m_widthResolution(widthResolution), m_heightResolution(heightResolution)
{

}

/**
* Full parameters constructor, all parameters settables.
* @param basicSettings Transducer settings.
* @param heightResolution Post-scan image height resolution in meters (distance between two pixels).
* @param widthResolution Post-scan image width resolution in meters (distance between two pixels).
*/
usImagePostScanSettings::usImagePostScanSettings(usTransducerSettings basicSettings,
                                                 double heightResolution, double widthResolution)
: usTransducerSettings(basicSettings), m_widthResolution(widthResolution), m_heightResolution(heightResolution)
{

}

/**
* Destructor.
*/
usImagePostScanSettings::~usImagePostScanSettings()
{

}

/**
* Assignement operator.
* @param other usImagePostScanSettings to copy.
*/
usImagePostScanSettings& usImagePostScanSettings::operator=(const usImagePostScanSettings& other)
{
  usTransducerSettings::operator=(other);
  m_heightResolution = other.getHeightResolution();
  m_widthResolution = other.getWidthResolution();

  return *this;
}

/**
* Comparaison operator.
* @param other usImagePostScanSettings to test.
*/
bool usImagePostScanSettings::operator==(const usImagePostScanSettings& other)
{
  return(usTransducerSettings::operator==(other) &&
    m_widthResolution == other.getWidthResolution() &&
    m_heightResolution == other.getHeightResolution());
}

/**
* Operator to print image informations on a stream.
* @param out ostream to write in.
* @param other usImagePostScanSettings with informations to print.
*/
VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usImagePostScanSettings &other)
{
  return out << static_cast<const usTransducerSettings &>(other) << 
      "Height resolution : " << other.getHeightResolution() <<
      "Width resolution : " << other.getWidthResolution();
}

/**
* Width resolution setter.
* @param widthResolution Width resolution to set.
*/
void usImagePostScanSettings::setWidthResolution(const double widthResolution)
{
  m_widthResolution = widthResolution;
}

/**
* Width resolution getter.
*/
double usImagePostScanSettings::getWidthResolution() const
{
  return m_widthResolution;
}

/**
* Height resolution setter.
* @param heightResolution Height resolution to set.
*/
void usImagePostScanSettings::setHeightResolution(const double heightResolution)
{
  m_heightResolution = heightResolution;
}

/**
* Height resolution getter.
*/
double usImagePostScanSettings::getHeightResolution() const
{
  return m_heightResolution;
}

/**
* Height resolution getter.
* @param postScanSettings Post-scan settings to set.
*/
void usImagePostScanSettings::setImageSettings(const usImagePostScanSettings& postScanSettings)
{
  *this = postScanSettings;
}

/**
* Settings getter.
*/
usImagePostScanSettings usImagePostScanSettings::getImageSettings() const
{
  return *this;
}
