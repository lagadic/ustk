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

#include <visp3/ustk_core/usImagePostScanSettings.h>

/**
 * @class usImagePostScan2D
 * @brief 2D postscan ultrasound image.
 *
 * This class represents a 2D ultrasound postscan frame.
 */
template<class T>
class usImagePostScan2D : public vpImage<T>, public usImagePostScanSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(unsigned int a_nubmer, unsigned int line_number, double probeRadius, double scanLinePitch, bool isProbeConvex, double axial_resolution);
  usImagePostScan2D(const usImagePostScan2D<T> &other);
  usImagePostScan2D(const vpImage<T> &other);
  usImagePostScan2D(const usImagePostScanSettings &other);
  usImagePostScan2D(const vpImage<T> &otherImage, const usImagePostScanSettings &otherSettings);
  ~usImagePostScan2D();

  usImagePostScan2D<T> & operator =(const usImagePostScan2D<T> &other);

  bool operator ==(const usImagePostScan2D<T> &other);

  void setData(const vpImage<T> &image);
  void setImageSettings(const usImagePostScanSettings &settings);
};

/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D() : vpImage<T>(), usImagePostScanSettings()
{

}

/**
* Full constructor, all parameters settables.
* @param a_nubmer number of A-samples in a line.
* @param line_number number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isProbeConvex Boolean to specify if the probe used was convex(true) or linear(false).
* @param axial_resolution Axial resolution of the image.
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(unsigned int a_nubmer, unsigned int line_number, double probeRadius, double scanLinePitch, bool isProbeConvex, double axial_resolution)
  : vpImage<T>(a_nubmer, line_number), usImagePostScanSettings(probeRadius, scanLinePitch, isProbeConvex, axial_resolution)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImagePostScan2D<T> &other) : vpImage<T>(other), usImagePostScanSettings(other)
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
* Constructor from usImagePostScanSettings.
* @param other usImagePostScanSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImagePostScanSettings &other) : usImagePostScanSettings(other)
{

}

/**
* Constructor from vpImage and usImagePostScanSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usImagePostScanSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const vpImage<T> &otherImage, const usImagePostScanSettings &otherSettings) : vpImage<T>(otherImage), usImagePostScanSettings(otherSettings)
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

  //from usImagePostScanSettings
  usImagePostScanSettings::operator=(other);

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
         usImagePostScanSettings::operator ==(other) &&
         m_widthResolution = other.getWidthResolution() &&
      m_heightResolution = other.getHeightResolution());
}

/**
* Setter for all imageSettings.
* @param settings Settings you want to copy.
*/
template<class T>
void usImagePostScan2D<T>::setImageSettings(const usImagePostScanSettings &settings)
{
  setProbeRadius(settings.getProbeRadius());
  setScanLinePitch(settings.getScanLinePitch());
  setImageConvex(settings.isImageConvex());
}

/**Setter for image.
* @param settings Settings you want to copy.
*/
template<class T>
void usImagePostScan2D<T>::setData(const vpImage<T> &image)
{
  vpImage<T>::operator=(image);
}
#endif // US_IMAGE_POSTSCAN_2D_H
