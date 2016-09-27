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

#include <visp3/ustk_core/usImageSettings3D.h>

/**
* @class usImagePostScan3D
* @brief 3D postscan ultrasound image.
*
* This class represents a 3D ultrasound postscan frame.
*/
template<class T>
class VISP_EXPORT usImagePostScan3D : public usImage3D<T>, public usImageSettings3D {
public:
  usImagePostScan3D();

  usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex);

  usImagePostScan3D(const usImagePostScan3D &other);

  usImagePostScan3D(const usImage3D<T> &other);

  usImagePostScan3D(const usImageSettings3D &other);

  usImagePostScan3D(const usImage3D<T> &otherImage, const usImageSettings3D &otherSettings);

  ~usImagePostScan3D();

  void setWidthResolution(double widthResolution);

  double getWidthResolution();

  void setHeightResolution(double widthResolution);

  double getHeightResolution();

private:
  double m_widthResolution;
  double m_heightResolution;
};


/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D() : usImage3D<T>(), usImageSettings3D()
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
* @param isImageConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isImageConvex, bool isMotorConvex)
  : usImage3D<T>(), usImageSettings3D(probeRadius, motorRadius, scanLinePitch, framePitch, isImageConvex, isMotorConvex)
{

}

/**
* Copy constructor from other usImagePostScan3D
* @param other usImagePostScan3D to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImagePostScan3D &other) : usImage3D<T>(other), usImageSettings3D(other)
{

}


/**
* Constructor from usImage3D
* @param other usImage3D<unsigned char> to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImage3D<T> &other) : usImage3D<T>(other)
{

}

/**
* Constructor from usImageSettings3D.
* @param other usImageSettings3D to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImageSettings3D &other) : usImageSettings3D(other)
{

}

/**
* Constructor from usImage3D and usImageSettings3D.
* @param otherImage usImage3D<unsigned char> to copy
* @param otherSettings usImageSettings3D to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImage3D<T> &otherImage, const usImageSettings3D &otherSettings) : usImage3D<T>(otherImage), usImageSettings3D(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePostScan3D<T>::~usImagePostScan3D() {}

/**
* Setter for width Resolution.
* @param widthResolution Width resolution (in meters) to set.
*/
template<class T>
void usImagePostScan3D<T>::setWidthResolution(double widthResolution) { m_widthResolution = widthResolution; }

/**
* Getter for width Resolution.
* @return widthResolution Width resolution (in meters).
*/
template<class T>
double usImagePostScan3D<T>::getWidthResolution() { return m_heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
template<class T>
void usImagePostScan3D<T>::setHeightResolution(double heightResolution) { m_heightResolution = heightResolution; }

/**
* Setter for width Resolution.
* @param heightResolution Height resolution (in meters) to set.
*/
template<class T>
double usImagePostScan3D<T>::getHeightResolution() { return m_heightResolution; }


#endif // US_IMAGE_POSTSCAN_3D_H
