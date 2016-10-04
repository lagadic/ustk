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

//std includes
#include <iostream>

//visp includes
#include <visp3/core/vpConfig.h>

//ustk includes
#include<visp3/ustk_core/usImagePostScan3DSettings.h>

/**
* Default constructor, all parameters set de default values.
*/
usImagePostScan3DSettings::usImagePostScan3DSettings() : usImageSettings3D(), m_widthResolution(0.0), m_heightResolution(0.0)
{

}

/**
* Full settings constructor, all settings availables.
*/
usImagePostScan3DSettings::usImagePostScan3DSettings(double probeRadius, double scanLinePitch, bool isProbeConvex,
  double motorRadius, double framePitch, bool isMotorConvex,
  double height_resolution, double width_resolution) : usImageSettings3D(probeRadius,motorRadius,scanLinePitch,framePitch,isProbeConvex,isMotorConvex), 
                                                       m_widthResolution(width_resolution), m_heightResolution(height_resolution)
{

}

/**
* Destructor.
*/
usImagePostScan3DSettings::~usImagePostScan3DSettings() {}

/**
* Assignement operator.
* @param other usImagePostScan3DSettings to copy.
*/
usImagePostScan3DSettings& usImagePostScan3DSettings::operator=(const usImagePostScan3DSettings& other)
{
  usImageSettings3D::operator=(other);
  m_widthResolution = other.getWidthResolution();
  m_heightResolution = other.getHeightResolution();

  return *this;
}

/**
* Comparaison operator.
* @param other usImagePostScan3DSettings to compare with.
*/
bool usImagePostScan3DSettings::operator==(const usImagePostScan3DSettings& other)
{
  return(usImageSettings3D::operator==(other) &&
    m_widthResolution == other.getWidthResolution() &&
    m_heightResolution == other.getHeightResolution());
}

/**
* Width resolution setter.
* @param widthResolution Width resloution to assign to the settings.
*/
void usImagePostScan3DSettings::setWidthResolution(const double widthResolution)
{
  m_widthResolution = widthResolution;
}

/**
* Width resolution getter.
*/
double usImagePostScan3DSettings::getWidthResolution() const
{
  return (m_widthResolution);
}

/**
* Height resolution setter. 
* @param heightResolution Height resolution to assign to the settings.
*/
void usImagePostScan3DSettings::setHeightResolution(const double heightResolution)
{
  m_heightResolution = heightResolution;
}

/**
* Height resolution getter.
*/
double usImagePostScan3DSettings::getHeightResolution() const
{
  return(m_heightResolution);
}