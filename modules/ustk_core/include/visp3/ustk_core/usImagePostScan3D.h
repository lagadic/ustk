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
* @brief 3D post-scan ultrasound image.
*/

#ifndef US_IMAGE_POSTSCAN_3D_H
#define US_IMAGE_POSTSCAN_3D_H

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usMotorSettings.h>
#include <visp3/ustk_core/usImagePostScanSettings.h>

/**
 @class usImagePostScan3D
 @brief This class represents a 3D ultrasound post-scan volume.
 @ingroup module_ustk_core

  <h3>Example</h3>
  The following example shows how to build a 3D post-scan ultrasound image from a usImage3D, and from acquisiton settings.

  \code


    #include <visp3/ustk_core/usImagePostScan3D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      unsigned int FN = 20;
      double probeRadius = 0.045;
      double scanLinePitch = 0.01;
      bool isTransducerConvex = true;
      double motorRadius = 0.07;
      double framePitch = 0.05;
      bool isMotorRotating = true;
      double heightResolution = 0.004;
      double widthResolution = 0.007;
      usMotorSettings   imageSettings(probeRadius, scanLinePitch, isTransducerConvex, motorRadius, framePitch, isMotorRotating, heightResolution, widthResolution);
      usImage3D<unsigned char> I(AN, LN, FN);
      usImagePostScan3D<unsigned char> postScan3d;
      postScan3d.setData(I);
      postScan3d.setImageSettings(imageSettings);
    }
  \endcode

*/
template<class T>
class usImagePostScan3D : public usImage3D<T>, public usImagePostScanSettings,public usMotorSettings {
public:
  usImagePostScan3D();
  usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN,
                    double probeRadius=0.0, double motorRadius=0.0, double scanLinePitch=0.0, double framePitch=0.0,
                    bool isImageConvex=false, bool isMotorRotating=false,
                    double heightResolution=0.0, double widthResolution=0.0);
  usImagePostScan3D(const usImagePostScan3D &other);
  usImagePostScan3D(const usImage3D<T> &otherImage, const usImagePostScanSettings &postScanSettings, const usMotorSettings &motorSettings);
  ~usImagePostScan3D();

  usImagePostScan3D<T> & operator =(const usImagePostScan3D<T> &other);

  bool operator ==(const usImagePostScan3D<T> &other);

  void setData(const usImage3D<T> &image3D);
};


/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D() : usImage3D<T>(), usImagePostScanSettings(), usMotorSettings()
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
* @param isTransducerConvex Boolean to specify if the image is acquired by a convex probe transducer (true) or by a linear probe transducer (false).
* @param isMotorRotating Boolean to specify if the image is acquired by a rotating  motor (true) or by a linear motor (false).
* @param heightResolution Image height resolution.
* @param widthResolution Image width resolution.
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch, bool isTransducerConvex, bool isMotorRotating, double heightResolution, double widthResolution)
  : usImage3D<T>(AN,LN,FN), usImagePostScanSettings(probeRadius, scanLinePitch, isTransducerConvex,heightResolution,widthResolution), usMotorSettings( motorRadius, framePitch, isMotorRotating)
{

}

/**
* Copy constructor from other usImagePostScan3D
* @param other usImagePostScan3D to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImagePostScan3D &other) : usImage3D<T>(other), usImagePostScanSettings(other), usMotorSettings(other)
{

}

/**
* Constructor from usImage3D and usMotorSettings.
* @param otherImage usImage3D<unsigned char> to copy
* @param postScanSettings usImagePostScanSettings to copy
* @param motorSettings usMotorSettings to copy
*/
template<class T>
usImagePostScan3D<T>::usImagePostScan3D(const usImage3D<T> &otherImage, const usImagePostScanSettings &postScanSettings, const usMotorSettings &motorSettings) : 
  usImage3D<T>(otherImage), usImagePostScanSettings(postScanSettings), usMotorSettings(motorSettings)
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
  usMotorSettings::operator =(other);

  return *this;
}

/**
* Comparaison operator.
*/
template<class T>
bool usImagePostScan3D<T>::operator == (usImagePostScan3D<T> const& other)
{
  return usImage3D<T>::operator ==(other) &&
        usImagePostScanSettings::operator==(other) &&
         usMotorSettings::operator ==(other);
}

/**
* Operator to print image informations on a stream.
*/
template<class T> std::ostream& operator<<(std::ostream& out, const usImagePostScan3D<T> &other)
{
  return out << static_cast<const usImage3D<T> &>(other) <<
    static_cast<const usImagePostScanSettings &>(other) <<
    static_cast<const usMotorSettings &>(other);
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
#endif // US_IMAGE_POSTSCAN_3D_H
