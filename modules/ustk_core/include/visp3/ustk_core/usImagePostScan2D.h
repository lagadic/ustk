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
 * @file usImagePostScan2D.h
 * @brief 2D postscan ultrasound image.
 */

#ifndef US_IMAGE_POSTSCAN_2D_H
#define US_IMAGE_POSTSCAN_2D_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImagePostScan2D
 * @brief 2D postscan ultrasound image.
 *
 * This class represents a 2D ultrasound postscan frame.
 */
template<class T>
class usImagePostScan2D : public vpImage<T>, public usImageSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex);
  usImagePostScan2D(const usImagePostScan2D<T> &other);
  usImagePostScan2D(const vpImage<T> &other);
  usImagePostScan2D(const usImageSettings &other);
  usImagePostScan2D(const vpImage<T> &otherImage, const usImageSettings &otherSettings);
  ~usImagePostScan2D();

  double getHeightResolution() const;
  double getWidthResolution() const;

  usImagePostScan2D<T> & operator =(const usImagePostScan2D<T> &other);

  bool operator ==(const usImagePostScan2D<T> &other);

  void setHeightResolution(double widthResolution);
  void setImageSettings(const usImageSettings &settings);
  void setWidthResolution(double widthResolution);

private:
  double m_widthResolution;
  double m_heightResolution;
};

/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D() : vpImage<T>(), usImageSettings()
{

}

/**
* Full constructor, all parameters settables.
* @param AN number of A-samples in a line.
* @param LN number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isConvex Boolean to specify if the probe used was convex(true) or linear(false).
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex)
  : vpImage<T>(AN,LN), usImageSettings(probeRadius, scanLinePitch, isConvex)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImagePostScan2D<T> &other) : vpImage<T>(other), usImageSettings(other)
{

}


/**
* Constructor from vpImage
* @param other vpImage<unsigned char> to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const vpImage<T> &other) : vpImage<T>(other)
{

}

/**
* Constructor from usImageSettings.
* @param other usImageSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImageSettings &other) : usImageSettings(other)
{

}

/**
* Constructor from vpImage and usImageSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usImageSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const vpImage<T> &otherImage, const usImageSettings &otherSettings) : vpImage<T>(otherImage), usImageSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePostScan2D<T>::~usImagePostScan2D() {}

/**
* Assignement operator.
*/
template<class T>
usImagePostScan2D<T> & usImagePostScan2D<T>::operator =(const usImagePostScan2D<T> &other)
{
  //from vpImage
  vpImage<T>::operator=(other);

  //from usImageSettings
  usImageSettings::operator=(other);

  //from this class
  m_widthResolution = other.getWidthResolution();
  m_heightResolution = other.getHeightResolution();

  return *this;
}

/**
* Comparaison operator.
*/
template<class T>
bool usImagePostScan2D<T>::operator ==(const usImagePostScan2D<T> &other)
{
  return(vpImage<T>::operator== (other) &&
         usImageSettings::operator ==(other) &&
         m_widthResolution = other.getWidthResolution() &&
      m_heightResolution = other.getHeightResolution());
}

/**
* Setter for width Resolution.
* @param widthResolution Width resolution (in meters) to set.
*/
template<class T>
void usImagePostScan2D<T>::setWidthResolution(double widthResolution) { m_widthResolution = widthResolution; }

/**
* Getter for width Resolution.
* @return widthResolution Width resolution (in meters).
*/
template<class T>
double usImagePostScan2D<T>::getWidthResolution() const { return m_heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
template<class T>
void usImagePostScan2D<T>::setHeightResolution(double heightResolution) { m_heightResolution = heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
template<class T>
double usImagePostScan2D<T>::getHeightResolution() const { return m_heightResolution; }

/**
* Setter for all imageSettings.
* @param settings Settings you want to copy.
*/
template<class T>
void usImagePostScan2D<T>::setImageSettings(const usImageSettings &settings)
{
  setProbeRadius(settings.getProbeRadius());
  setScanLinePitch(settings.getScanLinePitch());
  setImageConvex(settings.isImageConvex());
}

#endif // US_IMAGE_POSTSCAN_2D_H
