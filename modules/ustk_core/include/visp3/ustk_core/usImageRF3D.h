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

/**
* @file usImageRF3D.h
* @brief 3D RF ultrasound image.
*/

#ifndef US_IMAGE_RF_3D_H
#define US_IMAGE_RF_3D_H

#include <cstring>

#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImageSettings3D.h>

/**
* @class usImageRF3D
* @brief 3D RF ultrasound image.
*
* This class represents a 3D ultrasound RF frame.
*/
template<class T>
class usImageRF3D : public usImage3D<T>, public usImageSettings3D {
public:

  usImageRF3D();
  usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN);
  usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN,
              double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
              bool isImageConvex, bool isMotorConvex);
  usImageRF3D(usImage3D<T> image3D, usImageSettings3D imageSettings);
  usImageRF3D(usImage3D<T> image3D);
  usImageRF3D(usImageSettings3D imageSettings);
  usImageRF3D(const usImageRF3D<T> &other);
  ~usImageRF3D();

  double getAxialResolution() const;
  unsigned int getAN() const;
  unsigned int getFN() const;
  unsigned int getLN() const;

  usImageRF3D<T>& operator=(const usImageRF3D<T> &other);
  bool operator==(const usImageRF3D<T> &other);

  void setAxialResolution(double axialResolution);

private:
  double m_axialResolution;
};

/**
* Basic constructor.
*/
template<class T>
usImageRF3D<T>::usImageRF3D() : usImage3D<T>(), usImageSettings3D()
{

}

/**
* Initializing constructor for image dimentions.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
* @param[in] FN number of frames.
*/
template<class T>
usImageRF3D<T>::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN)
  : usImage3D<T>(AN, LN, FN), usImageSettings3D()
{

}

/**
* Full initializing constructor.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
* @param[in] FN number of frames.
* @param[in] probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param[in] motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param[in] scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] isImageConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param[in] isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
*/
template<class T>
usImageRF3D<T>::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                            bool isImageConvex, bool isMotorConvex)
  : usImage3D<T>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, scanLinePitch, framePitch, isImageConvex, isMotorConvex)
{

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
* @param imageSettings usImageSettings3D to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(usImage3D<T> image3D, usImageSettings3D imageSettings) : usImage3D<T>(image3D), usImageSettings3D(imageSettings) {

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(usImage3D<T> image3D) : usImage3D<T>(image3D) {

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param imageSettings usImageSettings3D to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(usImageSettings3D imageSettings) : usImageSettings3D(imageSettings) {

}

/**
* Copy constructor.
* @param other usImageRF3D to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(const usImageRF3D& other)
  : usImage3D<T>(other), usImageSettings3D(other)
{

}

/**
* Destructor.
*/
template<class T>
usImageRF3D<T>::~usImageRF3D()
{

}

/**
* Copy operator.
*/
template<class T>
usImageRF3D<T>& usImageRF3D<T>::operator=(const usImageRF3D<T> &other)
{
  //from vpImage
  usImage3D<T>::operator=(other);

  //from usImageSettings
  usImageSettings3D::operator=(other);

  //from this class
  m_axialResolution = other.getAxialResolution();
}

/**
* Comparaison operator.
*/
template<class T>
bool usImageRF3D<T>::operator==(const usImageRF3D<T> &other)
{
  return(usImage3D<T>::operator== (other) &&
         usImageSettings3D::operator ==(other) &&
         m_axialResolution == other.getAxialResolution());
}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
template<class T>
unsigned int usImageRF3D<T>::getAN() const { return usImage3D<T>::getDimX(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
template<class T>
unsigned int  usImageRF3D<T>::getLN() const { return usImage3D<T>::getDimY(); }

/**
* Get the number of frames.
* @return FN number of frames.
*/
template<class T>
unsigned int  usImageRF3D<T>::getFN() const { return usImage3D<T>::getDimZ(); }

/**
* Setter for axial Resolution.
* @param axialResolution Axial resolution (in meters) to set.
*/
template<class T>
void usImageRF3D<T>::setAxialResolution(double axialResolution) { m_axialResolution = axialResolution; }

/**
* Getter for axial Resolution.
* @return Axial resolution (in meters).
*/
template<class T>
double usImageRF3D<T>::getAxialResolution() const { return m_axialResolution; }

#endif // US_IMAGE_RF_3D_H
