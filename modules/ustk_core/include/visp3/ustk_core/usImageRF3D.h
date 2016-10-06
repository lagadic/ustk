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

#include <visp3/ustk_core/usImagePreScan3DSettings.h>

/*!
 @class usImageRF3D
 @brief 3D RF ultrasound image.
 This class represents a 3D ultrasound RF volume.

  <h3>Example</h3>
  The following example shows how to build a RF3D ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImageRF3D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      unsigned int FN = 30;
      double probeRadius = 0.0006;
      double scanLinePitch = 0.0007;
      bool isTransducerConvex = true;
      double motorRadius = 0.004;
      double framePitch = 0.06;
      bool isMotorRotating = true;
      double axialResolution = 0.001;
      usImagePreScan3DSettings  imageSettings(probeRadius, scanLinePitch, isTransducerConvex, motorRadius, framePitch, isMotorRotating, axialResolution);
      usImage3D<unsigned char> I(AN, LN, FN);
      usImageRF3D<unsigned char> rf3d;
      rf3d.setData(I);
      rf3d.setImageSettings(imageSettings);
    }
  \endcode

*/
template<class T>
class usImageRF3D : public usImage3D<T>, public usImagePreScan3DSettings {
public:

  usImageRF3D();
  usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN,
              double probeRadius=0.0, double motorRadius=0.0, double scanLinePitch=0.0, double framePitch=0.0,
              bool isImageConvex=false, bool isMotorRotating=false, double axial_resolution=0.0);
  usImageRF3D(usImage3D<T> image3D, usImagePreScan3DSettings imageSettings);
  usImageRF3D(usImage3D<T> image3D);
  usImageRF3D(usImagePreScan3DSettings imageSettings);
  usImageRF3D(const usImageRF3D<T> &other);
  ~usImageRF3D();

  usImageRF3D<T>& operator=(const usImageRF3D<T> &other);
  bool operator==(const usImageRF3D<T> &other);
  std::ostream& operator<<(std::ostream& out);

  void setData(const usImage3D<T> &image);
  void setImageSettings(const usImagePreScan3DSettings &settings);
};

/**
* Basic constructor.
*/
template<class T>
usImageRF3D<T>::usImageRF3D() : usImage3D<T>(), usImagePreScan3DSettings()
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
* @param[in] isTransducerConvex Boolean to specify if the image is acquired by a convex probe(true) or by a linear probe (false).
* @param[in] isMotorRotating Boolean to specify if the image is acquired by a rotating  motor(true) or by a linear motor (false).
* @param[in] axial_resolution The distance (in meters) between 2 successive pixels acquired along a scanline.
*/
template<class T>
usImageRF3D<T>::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                            bool isTransducerConvex, bool isMotorRotating, double axial_resolution)
  : usImage3D<T>(AN, LN, FN), usImagePreScan3DSettings(probeRadius, motorRadius, scanLinePitch, framePitch, isTransducerConvex, isMotorRotating, axial_resolution)
{

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
* @param imageSettings usImagePreScan3DSettings to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(usImage3D<T> image3D, usImagePreScan3DSettings imageSettings) : usImage3D<T>(image3D), usImagePreScan3DSettings(imageSettings) {

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
* @param imageSettings usImagePreScan3DSettings to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(usImagePreScan3DSettings imageSettings) : usImagePreScan3DSettings(imageSettings) {

}

/**
* Copy constructor.
* @param other usImageRF3D to copy
*/
template<class T>
usImageRF3D<T>::usImageRF3D(const usImageRF3D& other)
  : usImage3D<T>(other), usImagePreScan3DSettings(other)
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
  usImagePreScan3DSettings::operator=(other);
}

/**
* Comparaison operator.
*/
template<class T>
bool usImageRF3D<T>::operator==(const usImageRF3D<T> &other)
{
  return(usImage3D<T>::operator== (other) &&
         usImagePreScan3DSettings::operator ==(other));
}

/**
* Information printing operator.
* @param out Stream you want to write in.
*/
template<class T>
std::ostream& usImageRF3D<T>::operator<<(std::ostream& out)
{
  return usImage3D<T>::operator<<(out) <<
    usImagePreScan3DSettings::operator<<(out);
}

/**
* Prints information in a stream.
*/
template<class T>
std::ostream& operator<<(std::ostream& out, const usImageRF3D<T> &other)
{
  return out << static_cast<const usImage3D<T> &>(other) <<
    static_cast<const usImagePreScan3DSettings &>(other);
}

/**
* Setter for image data.
* @param image The image to set.
*/
template<class T>
void usImageRF3D<T>::setData(const usImage3D<T> &image)
{
  usImage3D<T>::operator=(image);
}

/**
* Setter for image settings.
* @param settings The settings to set.
*/
template<class T>
void usImageRF3D<T>::setImageSettings(const usImagePreScan3DSettings &settings)
{
  usImagePreScan3DSettings::operator=(settings);
}
#endif // US_IMAGE_RF_3D_H
