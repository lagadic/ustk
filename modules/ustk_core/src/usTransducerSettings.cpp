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
  : m_transducerRadius(0.0f), m_scanLinePitch(0.0f), m_scanLineNumber(0), m_isTransducerConvex(true),
    m_scanLineNumberIsSet(false) {}

/**
* Full constructor with all the settings availables:
* @param transducerRadius Distance between the center point of the transducer and the first pixel arc acquired.
* Value in meters (m).
* @param scanLinePitch Radius or distance between 2 successives scan lines acquired by the probe transducer; in radians (rad)
* if the probe is convex, or in meters (m) if the probe is linear.
* @param scanLineNumber Number of scan lines acquired by the probe transducer.
* @param transducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param depth Distance in meters between first and last pixel of a scan line.
*/
usTransducerSettings::usTransducerSettings(double transducerRadius, double scanLinePitch,
                                           unsigned int scanLineNumber, bool transducerConvex,
                                           double depth)
  : m_transducerRadius(transducerRadius), m_scanLinePitch(scanLinePitch),
    m_scanLineNumber(scanLineNumber), m_isTransducerConvex(transducerConvex),
    m_depth(depth), m_scanLineNumberIsSet(true)
{}

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
* @param other Settings you want to copy.
*/
usTransducerSettings& usTransducerSettings::operator=(const usTransducerSettings& other)
{
  m_transducerRadius = other.getTransducerRadius();
  m_scanLinePitch = other.getScanLinePitch();
  m_scanLineNumber = other.getScanLineNumber();
  m_isTransducerConvex = other.isTransducerConvex();
  m_scanLineNumberIsSet = other.scanLineNumberIsSet();
  m_depth = other.getDepth();

  return *this;
}

/**
* Compare two probe transducer settings.
* @return True if the settings are the same, false otherwise.
*/
bool usTransducerSettings::operator==(usTransducerSettings const& other)
{
  return ( this->getTransducerRadius() == other.getTransducerRadius() &&
           this->getScanLinePitch() == other.getScanLinePitch() &&
           this->getScanLineNumber() == other.getScanLineNumber() &&
           this->isTransducerConvex() == other.isTransducerConvex());
}

/*!
  Print transducer information in a ostream.
  Usage example:
  \code
  usTransducerSettings myTransducerSettings;
  std::cout << myTransducerSettings << std::endl;
  \endcode
*/
VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other)
{
  out << "transducer radius: " << other.getTransducerRadius() << std::endl;
  if (other.isTransducerConvex())
    out << "scan line pitch angle: " << other.getScanLinePitch() << std::endl;
  else
    out << "scan line pitch distance: " << other.getScanLinePitch() << std::endl;
  out << "scan line number : " << other.getScanLineNumber() << std::endl
      << "convex probe used: " << other.isTransducerConvex() << std::endl
      << "depth : " << other.getDepth() << std::endl;
  return out;
}

//Image settings getters/setters

/**
* Set the probe transducer radius (m).
* @param transducerRadius Probe transducer radius in meters. The probe transducer radius is set to 0 in case of a linear
* probe. See isTransducerConvex() and setProbeConvex(bool) for more information.
*/
void usTransducerSettings::setTransducerRadius(const double transducerRadius) { m_transducerRadius = transducerRadius; }

/**
* Get the probe transducer radius (m).
* @return Transducer radius in meters. The transducer radius is set to 0 if the probe is linear.
*/
double usTransducerSettings::getTransducerRadius() const { return m_transducerRadius; }

/**
* Set the scan line pitch.
* @param scanLinePitch If the probe transducer is convex, this parameters refers to the angle in radians between
* two successive scan lines acquired by the transducer. If the probe is linear, this parameters refers to the distance
* in meters between two successive scan lines acquired by the transducer.
*/
void usTransducerSettings::setScanLinePitch(const double scanLinePitch) { m_scanLinePitch = scanLinePitch; }

/**
* Get the scan line pitch (m).
*
* @return
* - When the probe transducer is convex, returns the angle in radians between two successive scan lines acquired
* by the transducer.
* - When the probe transducer is linear, returns the distance in meters between two successive scan lines acquired
* by the transducer.
*/
double usTransducerSettings::getScanLinePitch() const { return m_scanLinePitch; }

/**
* Set the probe transducer type.
* @param isTransducerConvex True if the transducer is convex, false if the transducer is linear.
* Sets the probe transducer radius to 0 in case of a linear transducer.
*/
void usTransducerSettings::setTransducerConvexity(const bool isTransducerConvex) {
  m_isTransducerConvex = isTransducerConvex;
  if (!isTransducerConvex) {
    m_transducerRadius = 0.0;
  }
}

/**
* Returns the probe transducer type.
* @return True if the transducer is convex, false if the transducer is linear.
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

/**
* Getter for the scan line number.
* @return Number of scan lines acquired by the probe transducer.
*/
unsigned int usTransducerSettings::getScanLineNumber() const
{
  return m_scanLineNumber;
}

/**
* Setter for the scan line number.
* @param scanLineNumber Number of scan lines acquired by the probe transducer.
*/
void usTransducerSettings::setScanLineNumber(unsigned int scanLineNumber)
{
  m_scanLineNumberIsSet = true;
  m_scanLineNumber =  scanLineNumber;
}

/**
* Getter for the probe name.
* @return Name of the probe.
*/
std::string usTransducerSettings::getProbeName() const
{
  return m_probeName;
}

/**
* Setter for the probe name.
* @param probeName Name of the probe.
*/
void usTransducerSettings::setProbeName(std::string probeName)
{
  m_probeName = probeName;
}

/**
* Getter for the transducer field of view (based on scan line number and pitch).
* @return The transducer field of view in radians if the probe is convex, in meters if it is linear.
*/
double usTransducerSettings::getFieldOfView() const
{
  if(!m_scanLineNumberIsSet)
    throw vpException(vpException::notInitialized, "The scan line number is not set, cannot determine the field of view");
  return m_scanLinePitch  * (double) (m_scanLineNumber -1);
}

/**
  Setter for the transducer field of view (updates the scan line pitch).
  @param fieldOfView The transducer field of view in radians if the transducer is convex, in meters
  if the transducer is linear.
  @warning Be sure to use setScanLineNumber() to update the scan line number before the field of view
  since this method computes the scan line pitch from the field of view and the scan line pitch.
\code
  usTransducerSettings transducerSettings;

  transducerSettings.setScanLineNumber(128);
  transducerSettings.setFieldOfView(vpMath::rad(57.0)); // field of view is 57 deg
\endcode
*/
void usTransducerSettings::setFieldOfView(double fieldOfView)
{
  if(!m_scanLineNumberIsSet)
    throw vpException(vpException::notInitialized, "The scan line number is not set, cannot determine the pitch from the field of view");
  m_scanLinePitch = fieldOfView / (double) (m_scanLineNumber - 1);
}

/**
* Getter to know if the scan line number is set (usefull in case of field of view setter call).
* @return Boolean to know if the scan line number is set or not.
*/
bool usTransducerSettings::scanLineNumberIsSet() const
{
  return m_scanLineNumberIsSet;
}

/**
* Setter for depth : distance in meters between first and last pixel in a scan line.
* @param depth Distance in meters.
*/
void usTransducerSettings::setDepth(double depth)
{
  m_depth = depth;
}

/**
* Setter for depth : distance in meters between first and last pixel in a scan line.
* @return Depth in meters.
*/
double usTransducerSettings::getDepth() const
{
  return m_depth;
}
