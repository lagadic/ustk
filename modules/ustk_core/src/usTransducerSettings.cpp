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
 * @file usTransducerSettings.cpp
 * @brief Generic ultrasound image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usTransducerSettings.h>

//ustk includes

/**
* Basic constructor, all settings set to default.
*/
usTransducerSettings::usTransducerSettings()
  : m_probeRadius(0.0f), m_scanLinePitch(0.0f), m_isTransducerConvex(true) {}

/**
* Full constructor with all the settings availables:
* @param probeRadius Distance between the center point of the probe and the first pixel arc acquired. Value in meters (m).
* @param scanLinePitch radius or distance between 2 successives acquisiton lines in the probe : in radians (rad) if the prove is convex, or in meters (m) if the probe is linear.
* @param isTransducerConvex Boolean to specify if the probe is convex or linear.
*/
usTransducerSettings::usTransducerSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex)
  : m_probeRadius(probeRadius), m_scanLinePitch(scanLinePitch), m_isTransducerConvex(isTransducerConvex) {}

/**
* Copy constructor.
* @param other Settings to copy.
*/
usTransducerSettings::usTransducerSettings(const usTransducerSettings &other)
{
  *this = other;
}

/**
* Destructor.
*/
usTransducerSettings::~usTransducerSettings() {}

/**
* Assignment operator.
* @param other settings you want to copy.
*/
usTransducerSettings& usTransducerSettings::operator=(const usTransducerSettings& other)
{
  m_probeRadius = other.getProbeRadius();
  m_scanLinePitch = other.getScanLinePitch();
  m_isTransducerConvex = other.isTransducerConvex();

  return *this;
}

/**
* Compare two image settings.
* @return True if the settings are the same, false otherwise.
*/
bool usTransducerSettings::operator==(usTransducerSettings const& other)
{
  return ( this->getProbeRadius() == other.getProbeRadius() &&
           this->getScanLinePitch() == other.getScanLinePitch() &&
           this->isTransducerConvex() == other.isTransducerConvex());
}

/**
* Print transducer informations in a ostream.
* Usage example : "std::cout << myTransducerSettings << std::endl;"
*/
VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other)
{
  return out << "probe radius: " << other.getProbeRadius() << std::endl
    << "line angle: " << other.getScanLinePitch() << std::endl
    << "convex probe used: " << other.isTransducerConvex() << std::endl;
}

//Image settings getters/setters

/**
* Set the probe radius (m).
* @param probeRadius Probe radius in meters. The probe radius is set to 0 in case of linear
* probe type (see isTransducerConvex() and setProbeConvex(bool) for more informations).
*/
void usTransducerSettings::setProbeRadius(const double probeRadius) { m_probeRadius = probeRadius; }

/**
* Get the probe radius (m).
* @return Probe radius in meters. The probe radius is set to 0 if the probe is linear.
*/
double usTransducerSettings::getProbeRadius() const { return m_probeRadius; }

/**
* Set the scan line pitch.
* @param scanLinePitch If the probe is convex, this parameters refers to the angle in radians between
* two successive lines acquired by the probe. If the probe is linear, this parameters refers to the distance
* in meters between two successive lines acquired by the probe.
*/
void usTransducerSettings::setScanLinePitch(const double scanLinePitch) { m_scanLinePitch = scanLinePitch; }

/**
* Get the scanline pitch (m).
*
* @return
* - When the probe is convex, returns the angle in radians between two successive lines acquired
* by the probe.
* - When the probe is linear, returns the distance in meters between two successive lines acquired
* by the probe.
*/
double usTransducerSettings::getScanLinePitch() const { return m_scanLinePitch; }

/**
* Set the probe type.
* @param isTransducerConvex True if the probe is convex, false if the probe is linear.
* Sets the probe radius to 0 in case of a linear probe.
*/
void usTransducerSettings::setTransducerConvexity(const bool isTransducerConvex) {
  m_isTransducerConvex = isTransducerConvex;
  if (!isTransducerConvex) {
    m_probeRadius = 0.0;
  }
}

/**
* Returns the probe type.
* @return True if the probe is convex, false if the probe is linear.
*/
bool usTransducerSettings::isTransducerConvex() const { return m_isTransducerConvex; }

/*!
 * Assignment operator.
 *
 * @param other Settings you want to assign.
 *
 * \sa operator=()
 */
void usTransducerSettings::setTransducerSettings(const usTransducerSettings& other)
{
  *this = other;
}
