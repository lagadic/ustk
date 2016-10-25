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
#include<visp3/ustk_core/usImagePreScanSettings.h>

/**
* Default constructor, all parameters set with default values.
*/
usImagePreScanSettings::usImagePreScanSettings()
  : usTransducerSettings(), m_axialResolution(0.0)
{

}

/**
* Copy constructor.
* @param other Image settings to copy.
*/
usImagePreScanSettings::usImagePreScanSettings(const usImagePreScanSettings &other)
  : usTransducerSettings(other), m_axialResolution(other.getAxialResolution())
{

}

/**
* Full settings constructor.
* @param transducerSettings Transducer settings.
* @param axialResolution Image axial resolution in meters (distance between two samples
* in a scanline).
*/
usImagePreScanSettings::usImagePreScanSettings(const usTransducerSettings &transducerSettings, double axialResolution)
  : usTransducerSettings(transducerSettings), m_axialResolution(axialResolution)
{

}

/**
* Copy constructor.
* @param preScanSettings Pre-scan settings to copy.
*/
void usImagePreScanSettings::setImageSettings(const usImagePreScanSettings& preScanSettings)
{
  *this = preScanSettings;
}

/**
* Destructor.
*/
usImagePreScanSettings::~usImagePreScanSettings() {}

/**
* Assignement operator.
* @param other usImagePreScanSettings to copy.
*/
usImagePreScanSettings& usImagePreScanSettings::operator=(const usImagePreScanSettings& other)
{
  usTransducerSettings::operator=(other);
  m_axialResolution = other.getAxialResolution();

  return *this;
}

/**
* Comparaison operator.
* @param other usImagePreScanSettings to compare with.
*/
bool usImagePreScanSettings::operator==(const usImagePreScanSettings& other)
{
  return(usTransducerSettings::operator==(other) &&
    m_axialResolution == other.getAxialResolution());
}

/**
* Prints information in a stream.
*/
template<class T> VISP_EXPORT
std::ostream& operator<<(std::ostream& out, const usImagePreScanSettings &other)
{
  return out << static_cast<const usTransducerSettings &>(other) <<
    "Axial resolution : " << other.getAxialResolution() << std::endl;
}

/**
* Axial resolution setter.
* @param axialResolution Axial resloution to assign to the settings.
*/
void usImagePreScanSettings::setAxialResolution(const double axialResolution)
{
  m_axialResolution = axialResolution;
}

/**
* Axial resolution getter.
*/
double usImagePreScanSettings::getAxialResolution() const
{
  return (m_axialResolution);
}



