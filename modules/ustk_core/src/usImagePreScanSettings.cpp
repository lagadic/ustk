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
* Default constructor, all parameters set de default values.
*/
usImagePreScanSettings::usImagePreScanSettings() : usImageSettings(), m_axialResolution(0.0)
{

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
  usImageSettings::operator=(other);
  m_axialResolution = other.getAxialResolution();

  return *this;
}

/**
* Comparaison operator.
* @param other usImagePreScanSettings to compare with.
*/
bool usImagePreScanSettings::operator==(const usImagePreScanSettings& other)
{
  return(usImageSettings::operator==(other) &&
    m_axialResolution == other.getAxialResolution());
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
* Width resolution getter.
*/
double usImagePreScanSettings::getAxialResolution() const
{
  return (m_axialResolution);
}



