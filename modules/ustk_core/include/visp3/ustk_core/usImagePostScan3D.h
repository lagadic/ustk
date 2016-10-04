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
* @file usImagePostScan3D.h
* @brief 3D postscan ultrasound image.
*/

#ifndef US_IMAGE_POSTSCAN_3D_H
#define US_IMAGE_POSTSCAN_3D_H

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImagePostScan3DSettings.h>

/**
* @class usImagePostScan3D
* @brief 3D postscan ultrasound image.
*
* This class represents a 3D ultrasound postscan frame.
*/
template<class T>
class usImagePostScan3D : public usImage3D<T>, public usImagePostScan3DSettings {
public:
  usImagePostScan3D();
  usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN,
                    double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                    bool isImageConvex, bool isMotorConvex,
                    double heightResolution, double widthResolution);
  usImagePostScan3D(const usImagePostScan3D &other);
  usImagePostScan3D(const usImage3D<T> &otherImage, const usImagePostScan3DSettings &otherSettings);
  ~usImagePostScan3D();

  usImagePostScan3D<T> & operator =(const usImagePostScan3D<T> &other);

  bool operator ==(const usImagePostScan3D<T> &other);

  void setData(const usImage3D<T> &image3D);
  void setSettings(const usImagePostScan3DSettings &postScan3DSettings);
};


/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D() : usImage3D<T>(), usImagePostScan3DSettings()
{

}

/**
* Complete constructor, all parameters availables.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
* @param FN Number of Frames.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isProbeConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
* @param heightResolution Image height resolution.
* @param widthResolution Image width resolution.
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isProbeConvex, bool isMotorConvex, double heightResolution, double widthResolution)
  : usImage3D<T>(), usImagePostScan3DSettings(probeRadius, motorRadius, scanLinePitch, framePitch, isProbeConvex, isMotorConvex, heightResolution, widthResolution)
{

}

/**
* Copy constructor from other usImagePostScan3D
* @param other usImagePostScan3D to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImagePostScan3D &other) : usImage3D<T>(other), usImagePostScan3DSettings(other)
{

}

/**
* Constructor from usImage3D and usImagePostScan3DSettings.
* @param otherImage usImage3D<unsigned char> to copy
* @param otherSettings usImagePostScan3DSettings to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImage3D<T> &otherImage, const usImagePostScan3DSettings &otherSettings) : usImage3D<T>(otherImage), usImagePostScan3DSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePostScan3D<T>::~usImagePostScan3D() {}

/**
* Assignement operator.
*/
template<class T>
usImagePostScan3D<T> & usImagePostScan3D<T>::operator =(const usImagePostScan3D<T> &other)
{
  //from usImage3D
  usImage3D<T>::operator =(other);

  //from usSettings3D
  usImagePostScan3DSettings::operator =(other);

  //from this class
  m_widthResolution = other.getWidthResolution();
  m_heightResolution = other.getHeightResolution();

  return *this;
}

/**
* Comparaison operator.
*/
template<class T>
bool usImagePostScan3D<T>::operator == (usImagePostScan3D<T> const& other)
{
  return(usImage3D<T>::operator ==(other) &&
         usImagePostScan3DSettings::operator ==(other) &&
         getWidthResolution() == other.getWidthResolution() &&
         getHeightResolution() == other.getHeightResolution());
}

/**
* Setter for image data.
* @param image3D Image you want to set.
*/
template<class T>
void usImagePostScan3D<T>::setData(const usImage3D<T> &image3D)
{
  usImage3D<T>::operator =(image3D);
}

/**
* Setter for image data.
* @param postScan3DSettings Image settings you want to set.
*/
template<class T>
void  usImagePostScan3D<T>::setSettings(const usImagePostScan3DSettings &postScan3DSettings)
{
  usImagePostScan3DSettings::operator=(postScan3DSettings);
}
#endif // US_IMAGE_POSTSCAN_3D_H
