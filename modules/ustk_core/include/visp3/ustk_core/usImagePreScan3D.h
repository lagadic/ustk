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
* This file is provided AS IS with NO WARRBModeSampleNumberTY OF BModeSampleNumberY KIND, INCLUDING THE
* WARRBModeSampleNumberTY OF DESIGN, MERCHBModeSampleNumberTABILITY BModeSampleNumberD FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/ustk_core/usImagePreScanSettings.h>

#include <visp3/ustk_core/usMotorSettings.h>

/*!
 @class usImagePreScan3D
 @brief 3D pre-scan ultrasound image.
 @ingroup module_ustk_core

 This class represents a 3D ultrasound pre-scan frame.

  <h3>Example</h3>
  The following example shows how to build a 3D pre-scan ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImagePreScan3D.h>

    int main()
    {
      // Update settings
      unsigned int BModeSampleNumber = 200;
      unsigned int scanLineNumber = 200;
      unsigned int frameNumber = 10;
      double probeRadius = 0.008;
      double scanLinePitch = 0.004;
      bool isTransducerConvex = true;
      double motorRadius = 0.0008;
      double framePitch = 0.05;
      usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;
      double axialResolution = 0.004;
      usImagePreScanSettings   imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      usMotorSettings motorSettings(motorRadius,framePitch,motorType);
      usImage3D<unsigned char> I(BModeSampleNumber, scanLineNumber, frameNumber);
      usImagePreScan3D<unsigned char> preScan3d;
      preScan3d.setData(I);
      preScan3d.setImageSettings(imageSettings);
      preScan3d.setMotorSettings(motorSettings);
    }
  \endcode
*/
template<class Type>
class usImagePreScan3D : public usImage3D<Type>, public usImagePreScanSettings, public usMotorSettings {
public:
  //default constructors
  usImagePreScan3D();
  //All parameters initialisation constructors
  usImagePreScan3D(unsigned int BModeSampleNumber, unsigned int scanLineNumber, unsigned int frameNumber,
                   double probeRadius=0.0, double motorRadius=0.0, double scanLinePitch=0.0, double framePitch=0.0,
                   bool isImageConvex=false, const usMotorType &motorType=usMotorSettings::LinearMotor, double axialResolution=0.0);
  //usImagePreScan3D copy constructor
  usImagePreScan3D(const usImagePreScan3D &other);
  //usImage3D copy constructor
  usImagePreScan3D(const usImage3D<Type> &other);
  //usImagePreScanSettings copy constructor
  usImagePreScan3D(const usImagePreScanSettings &other);
  //copy constructor from usImage3D and usImagePreScanSettings
  usImagePreScan3D(const usImage3D<Type> &other, const usImagePreScanSettings &otherSettings, const usMotorSettings &motorSettings);
  //destructor
  virtual ~usImagePreScan3D();

  unsigned int getBModeSampleNumber() const;
  unsigned int getFrameNumber() const;
  unsigned int getScanLineNumber() const;

  //assignement
  usImagePreScan3D<Type>& operator=(const usImagePreScan3D<Type> &other);
  //comparison
  bool operator==(const usImagePreScan3D<Type> &other);

  void setData(const usImage3D<Type> &image);
  void setImageSettings(const usImagePreScanSettings &settings);
  void setMotorSettings(const usMotorSettings &settings);
};

/**
* Basic constructor, all settings set to default.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D() : usImage3D<Type>(), usImagePreScanSettings()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param BModeSampleNumber B-mode samples in a line (in px).
* @param scanLineNumber Number of scan lines (in px).
* @param frameNumber Number of Frames.
* @param probeRadius Radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param motorType usMotorType to specify if the image is acquired by a linear motor (LinearMotor),
* by a small angle rotation motor (TiltingMotor), or by a 360&deg; roatation motor (RotationalMotor).
* @param isTransducerConvex Transducer type : true for convex transducer, false for linear transducer.
* @param axialResolution Axial resolution of the image.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(unsigned int BModeSampleNumber, unsigned int scanLineNumber,
                                         unsigned int frameNumber, double probeRadius, double motorRadius,
                                         double scanLinePitch, double framePitch,
                                         bool isTransducerConvex, const usMotorSettings::usMotorType &motorType,
                                         double axialResolution)
  : usImage3D<Type>(BModeSampleNumber, scanLineNumber, frameNumber),
    usImagePreScanSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution),
    usMotorSettings(motorRadius, framePitch, motorType)
{

}

/**
* Copy constructor. For double image type.
* @param other usImagePreScan3D image you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImagePreScan3D &other)
  : usImage3D<Type>(other), usImagePreScanSettings(other), usMotorSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param other usImage3D<double> image you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImage3D<Type> &other) : usImage3D<Type>(other)
{

}

/**
* Copy constructor.
* @param other usImagePreScanSettings you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImagePreScanSettings &other) : usImagePreScanSettings(other)
{

}

/**
* Copy constructor. For double image type
* @param other usImage3D you want to copy.
* @param otherSettings usImagePreScanSettings you want to copy.
* @param motorSettings usMotorSettings you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImage3D<Type> &other, const usImagePreScanSettings &otherSettings,
const usMotorSettings &motorSettings)
: usImage3D<Type>(other), usImagePreScanSettings(otherSettings), usMotorSettings(motorSettings)
{

}

/**
* Destructor.
*/
template<class Type>
usImagePreScan3D<Type>::~usImagePreScan3D() {}

/**
* Copy operator.
*/
template<class Type>
usImagePreScan3D<Type>& usImagePreScan3D<Type>::operator=(const usImagePreScan3D<Type> &other)
{
  //from vpImage
  usImage3D<Type>::operator=(other);

  //from usImageSettings
  usImagePreScanSettings::operator=(other);
}

/**
* Comparaison operator.
*/
template<class Type>
bool usImagePreScan3D<Type>::operator==(const usImagePreScan3D<Type> &other)
{
  return(usImage3D<Type>::operator== (other) &&
         usImagePreScanSettings::operator== (other));
}

/**
* Prints information in a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePreScan3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
    static_cast<const usImagePreScanSettings &>(other);
}

/**
* Gets the number of B-mode samples along a scanline (volume height in px).
*/
template<class Type>
unsigned int usImagePreScan3D<Type>::getBModeSampleNumber() const {
  return usImage3D<Type>::getDimY();
}


/**
* Gets the number of scanlines (volume width in px).
*/
template<class Type>
unsigned int usImagePreScan3D<Type>::getScanLineNumber() const {
  return usImage3D<Type>::getDimX();
}

/**
* Gets the number of 2D frames in the volume.
*/
template<class Type>
unsigned int usImagePreScan3D<Type>::getFrameNumber() const {
  return usImage3D<Type>::getDimZ();
}

/**
* Setter for the image data.
* @param image The image to set.
*/
template<class Type>
void usImagePreScan3D<Type>::setData(const usImage3D<Type> &image)
{
  usImage3D<Type>::operator=(image);
}

/**
* Setter for the image settings.
* @param settings The new image settings.
*/
template<class Type>
void usImagePreScan3D<Type>::setImageSettings(const usImagePreScanSettings &settings)
{
  usImagePreScanSettings::operator=(settings);
}

/**
* Setter for the motor settings.
* @param settings The new motor settings.
*/
template<class Type>
void usImagePreScan3D<Type>::setMotorSettings(const usMotorSettings &settings)
{
  usMotorSettings::operator=(settings);
}
#endif // US_IMAGE_PRESCAN_3D_H
