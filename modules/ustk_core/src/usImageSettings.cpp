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

/**
 * @file usImageSettings.cpp
 * @brief Generic ultrasound image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usImageSettings.h>

//ustk includes

/**
* Basic constructor, all settings set to default.
*/
usImageSettings::usImageSettings() : m_probeRadius(0.0f), m_scanLinePitch(0.0f), m_isImageConvex(true) {}

/**
* Full constructor with all the settings availables:
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired. Value in meters (m).
* @param[in] scanLinePitch radius or distance between 2 successives acquisiton lines in the probe : in radians (rad) if the prove is convex, or in meters (m) if the probe is linear.
* @param[in] isConvex Boolean to specify if the probe is convex or linear.
*/
usImageSettings::usImageSettings(double probeRadius, double scanLinePitch, bool isConvex) : m_probeRadius(probeRadius), m_scanLinePitch(scanLinePitch), m_isImageConvex(isConvex) {}

/**
* Copy constructor.
* @param[in] other Settings to copy.
*/
usImageSettings::usImageSettings(const usImageSettings &other)
{
  *this = other;
}

/**
* Destructor.
*/
usImageSettings::~usImageSettings() {}

/**
* Assignment operator.
* @param[in] other usImageSettings you want to copy.
*/
usImageSettings& usImageSettings::operator=(const usImageSettings& other)
{
  m_probeRadius = other.getProbeRadius();
  m_scanLinePitch = other.getScanLinePitch();
  setImageConvex(other.isImageConvex());

  return *this;
}

/**
* Compare two image settings.
* @return True if the settings are the same, false otherwise.
*/
bool usImageSettings::operator==(usImageSettings const& other)
{
  return ( this->getProbeRadius() == other.getProbeRadius() &&
           this->getScanLinePitch() == other.getScanLinePitch() &&
           this->isImageConvex() == other.isImageConvex());
}
//Image settings getters/setters

/**
* Set the probe radius (m).
* @param[in] probeRadius Probe radius in meters. The probe radius is set to 0 in case of linear
* probe type (see isImageConvex() and setImageConvex(bool) for more informations).
*/
void usImageSettings::setProbeRadius(const double probeRadius) { m_probeRadius = probeRadius; }

/**
* Get the probe radius (m).
* @return Probe radius in meters. The probe radius is set to 0 if the probe is linear.
*/
double usImageSettings::getProbeRadius() const { return m_probeRadius; }

/**
* Set the scan line pitch.
* @param[in] scanLinePitch If the probe is convex, this parameters refers to the angle in radians between
* two successive lines acquired by the probe. If the probe is linear, this parameters refers to the distance
* in meters between two successive lines acquired by the probe.
*/
void usImageSettings::setScanLinePitch(const double scanLinePitch) { m_scanLinePitch = scanLinePitch; }

/**
* Get the scanline pitch (m).
*
* @return
* - When the probe is convex, returns the angle in radians between two successive lines acquired
* by the probe.
* - When the probe is linear, returns the distance in meters between two successive lines acquired
* by the probe.
*/
double usImageSettings::getScanLinePitch() const { return m_scanLinePitch; }

/**
* Set the probe type.
* @param[in] isConvex True if the probe is convex, false if the probe is linear.
* Sets the probe radius to 0 in case of a linear probe.
*/
void usImageSettings::setImageConvex(const bool isConvex) {
  m_isImageConvex = isConvex;
  if (!isConvex) {
    m_probeRadius = 0.0f;
  }
}

/**
* Returns the probe type.
* @return True if the probe is convex, false if the probe is linear.
*/
bool usImageSettings::isImageConvex() const { return m_isImageConvex; }

/**
* Print probe settings information.
*/
void usImageSettings::printProbeSettings()
{
  std::cout << "probe radius: " << m_probeRadius << std::endl
            << "line angle: " << m_scanLinePitch << std::endl
            << "convex probe used: " << m_isImageConvex << std::endl;
}

void usImageSettings::setImageSettings(const usImageSettings& other)
{
  *this = other;
}
