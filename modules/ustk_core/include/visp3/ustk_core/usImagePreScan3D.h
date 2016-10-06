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
* @file usImagePreScan3D.h
* @brief 3D pre-scan ultrasound image.
*/

#ifndef US_IMAGE_PRESCAN_3D_H
#define US_IMAGE_PRESCAN_3D_H

#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImagePreScan3DSettings.h>

/*!
 @class usImagePreScan3D
 @brief 3D pre-scan ultrasound image.

 This class represents a 3D ultrasound pre-scan frame.

  <h3>Example</h3>
  The following example shows how to build a 3D pre-scan ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImagePreScan3D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      unsigned int FN = 10;
      double probeRadius = 0.008;
      double scanLinePitch = 0.004;
      bool isTransducerConvex = true;
      double motorRadius = 0.0008;
      double framePitch = 0.05;
      bool isMotorRotating = true;
      double axialResolution = 0.004;
      usImagePreScan3DSettings   imageSettings(probeRadius, scanLinePitch, isTransducerConvex, motorRadius, framePitch, isMotorRotating, axialResolution);
      usImage3D<unsigned char> I(AN, LN, FN);
      usImagePreScan3D<unsigned char> preScan3d;
      preScan3d.setData(I);
      preScan3d.setImageSettings(imageSettings);
    }
  \endcode
*/
template<class T>
class usImagePreScan3D : public usImage3D<T>, public usImagePreScan3DSettings {
public:
  //default constructors
  usImagePreScan3D();
  //All parameters initialisation constructors
  usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN,
                   double probeRadius=0.0, double motorRadius=0.0, double scanLinePitch=0.0, double framePitch=0.0,
                   bool isImageConvex=false, bool isMotorRotating=false, double axial_resolution=0.0);
  //usImagePreScan3D copy constructor
  usImagePreScan3D(const usImagePreScan3D &other);
  //usImage3D copy constructor
  usImagePreScan3D(const usImage3D<T> &other);
  //usImagePreScan3DSettings copy constructor
  usImagePreScan3D(const usImagePreScan3DSettings &other);
  //copy constructor from usImage3D and usImagePreScan3DSettings
  usImagePreScan3D(const usImage3D<T> &other, const usImagePreScan3DSettings &otherSettings);
  //destructor
  ~usImagePreScan3D();

  //assignement
  usImagePreScan3D<T>& operator=(const usImagePreScan3D<T> &other);
  //comparison
  bool operator==(const usImagePreScan3D<T> &other);

  void setData(const usImage3D<T> &image);
  void setImageSettings(const usImagePreScan3DSettings &settings);
};

/**
* Basic constructor, all settings set to default.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D() : usImage3D<T>(), usImagePreScan3DSettings()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
* @param FN Number of Frames.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isTransducerConvex Boolean to specify if the image is acquired by a convex probe transducer (true) or by a linear probe transducer (false).
* @param isMotorRotating Boolean to specify if the image is acquired by a rotating  motor (true) or by a linear motor (false).
* @param axial_resolution Axial resolution of the image.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                                      bool isTransducerConvex, bool isMotorRotating, double axial_resolution) :
  usImage3D<T>(AN, LN, FN), usImagePreScan3DSettings(probeRadius, motorRadius, scanLinePitch, framePitch, isTransducerConvex, isMotorRotating, axial_resolution)
{

}

/**
* Copy constructor. For double image type.
* @param other usImagePreScan3D image you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImagePreScan3D &other) :
  usImage3D<T>(other), usImagePreScan3DSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param other usImage3D<double> image you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImage3D<T> &other) : usImage3D<T>(other)
{

}

/**
* Copy constructor.
* @param other usImagePreScan3DSettings you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImagePreScan3DSettings &other) : usImagePreScan3DSettings(other)
{

}

/**
* Copy constructor. For double image type
* @param other usImage3D you want to copy.
* @param otherSettings usImagePreScan3DSettings you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImage3D<T> &other, const usImagePreScan3DSettings &otherSettings) :
  usImage3D<T>(other), usImagePreScan3DSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePreScan3D<T>::~usImagePreScan3D() {}

/**
* Copy operator.
*/
template<class T>
usImagePreScan3D<T>& usImagePreScan3D<T>::operator=(const usImagePreScan3D<T> &other)
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
bool usImagePreScan3D<T>::operator==(const usImagePreScan3D<T> &other)
{
  return(usImage3D<T>::operator== (other) &&
         usImagePreScan3DSettings::operator ==(other));
}

/**
* Prints information in a stream.
*/
template<class T>
std::ostream& operator<<(std::ostream& out, const usImagePreScan3D<T> &other)
{
  return out << static_cast<const usImage3D<T> &>(other) <<
    static_cast<const usImagePreScan3DSettings &>(other);
}

/**
* Setter for the image data.
* @param image The image to set.
*/
template<class T>
void usImagePreScan3D<T>::setData(const usImage3D<T> &image)
{
  usImage3D<T>::operator=(image);
}

/**
* Setter for the image settings.
* @param settings The new image settings.
*/
template<class T>
void usImagePreScan3D<T>::setImageSettings(const usImagePreScan3DSettings &settings)
{
  usImagePreScan3DSettings::operator=(settings);
}
#endif // US_IMAGE_PRESCAN_3D_H
