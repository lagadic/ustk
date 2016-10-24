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
* This file is provided AS IS with NO WARRdimXTY OF dimXY KIND, INCLUDING THE
* WARRdimXTY OF DESIGN, MERCHdimXTABILITY dimXD FITNESS FOR A PARTICULAR PURPOSE.
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
      unsigned int dimX = 200;
      unsigned int dimY = 200;
      unsigned int dimZ = 20;
      double probeRadius = 0.045;
      double scanLinePitch = 0.01;
      bool isTransducerConvex = true;
      double motorRadius = 0.07;
      double framePitch = 0.05;
      usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;
      double heightResolution = 0.004;
      double widthResolution = 0.007;
      usImagePostScanSettings   imageSettings(probeRadius, scanLinePitch, isTransducerConvex, heightResolution, widthResolution);
      usMotorSettings motorSettings(motorRadius,framePitch,motorType);
      usImage3D<unsigned char> I(dimX, dimY, dimZ);
      usImagePostScan3D<unsigned char> postScan3d;
      postScan3d.setData(I);
      postScan3d.setImageSettings(imageSettings);
      postScan3d.setMotorSettings(motorSettings);
    }
  \endcode

*/
template<class Type>
class usImagePostScan3D : public usImage3D<Type>, public usTransducerSettings ,public usMotorSettings {
public:
  usImagePostScan3D();
  usImagePostScan3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ,
                    double probeRadius, double motorRadius, double scanLinePitch,
                    double framePitch, bool isTransducerConvex, const usMotorSettings::usMotorType &motorType,
                    double spacingX, double spacingY, double spacingZ);

  usImagePostScan3D(const usImagePostScan3D &other);
  usImagePostScan3D(const usImage3D<Type> &otherImage, const usTransducerSettings &imageSettings,
                    const usMotorSettings &motorSettings);
  virtual ~usImagePostScan3D();

  usImagePostScan3D<Type> & operator =(const usImagePostScan3D<Type> &other);

  bool operator ==(const usImagePostScan3D<Type> &other);

  void setData(const usImage3D<Type> &image3D);

};


/**
* Basic constructor, all parameters set to default values
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D() : usImage3D<Type>(), usTransducerSettings(), usMotorSettings()
{

}

/**
* Complete constructor, all parameters availables.
* @param dimX Number of voxels along x axis.
* @param dimY Number of voxels along y axis.
* @param dimZ Number of voxales along z axis.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isTransducerConvex Boolean to specify if the image is acquired by a convex probe transducer (true) or by a linear probe transducer (false).
* @param motorType usMotorType to specify if the image is acquired by a linear motor (LinearMotor),
* by a small angle rotation motor (TiltingMotor), or by a 360&deg; roatation motor (RotationalMotor).
* @param spacingX Resolution in x axis (in meters).
* @param spacingY Resolution in y axis (in meters).
* @param spacingZ Resolution in z axis (in meters).
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ,
                                        double probeRadius, double motorRadius, double scanLinePitch,
                                        double framePitch, bool isTransducerConvex,
                                        const usMotorSettings::usMotorType &motorType,
                                        double spacingX, double spacingY, double spacingZ)
  : usImage3D<Type>(dimX, dimY, dimZ, spacingX, spacingY, spacingZ),
    usTransducerSettings(probeRadius, scanLinePitch, isTransducerConvex),
    usMotorSettings( motorRadius, framePitch, motorType),
{

}

/**
* Copy constructor from other usImagePostScan3D
* @param other usImagePostScan3D to copy
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D(const usImagePostScan3D &other)
  : usImage3D<Type>(other), usTransducerSettings(other), usMotorSettings(other)
{

}

/**
* Constructor from usImage3D and usMotorSettings.
* @param otherImage usImage3D<unsigned char> to copy
* @param transducerSettings usTransducerSettings to copy
* @param motorSettings usMotorSettings to copy
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D(const usImage3D<Type> &otherImage, const usTransducerSettings &transducerSettings,
  const usMotorSettings &motorSettings)
: usImage3D<Type>(otherImage), usTransducerSettings(transducerSettings), usMotorSettings(motorSettings)
{

}

/**
* Destructor.
*/
template<class Type>
usImagePostScan3D<Type>::~usImagePostScan3D() {}

/**
* Assignement operator.
*/
template<class Type>
usImagePostScan3D<Type> & usImagePostScan3D<Type>::operator =(const usImagePostScan3D<Type> &other)
{
  //from usImage3D
  usImage3D<Type>::operator =(other);

  //from usSettings3D
  usMotorSettings::operator =(other);

  return *this;
}

/**
* Comparaison operator.
*/
template<class Type>
bool usImagePostScan3D<Type>::operator == (usImagePostScan3D<Type> const& other)
{
  return usImage3D<Type>::operator ==(other) &&
    usTransducerSettings::operator==(other) &&
         usMotorSettings::operator ==(other);
}

/**
* Operator to print image informations on a stream.
*/
template<class Type> std::ostream& operator<<(std::ostream& out, const usImagePostScan3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
    static_cast<const usTransducerSettings &>(other) <<
    static_cast<const usMotorSettings &>(other);
}

/**
* Setter for image data.
* @param image3D Image you want to set.
*/
template<class Type>
void usImagePostScan3D<Type>::setData(const usImage3D<Type> &image3D)
{
  usImage3D<Type>::operator =(image3D);
}
#endif // US_IMAGE_POSTSCAN_3D_H
