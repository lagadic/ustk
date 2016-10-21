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
* This file is provided AS IS with NO WARRRFSampleNumberTY OF RFSampleNumberY KIND, INCLUDING THE
* WARRRFSampleNumberTY OF DESIGN, MERCHRFSampleNumberTABILITY RFSampleNumberD FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/ustk_core/usImagePreScanSettings.h>
#include <visp3/ustk_core/usMotorSettings.h>

/*!
 @class usImageRF3D
 @brief 3D RF ultrasound image.
 @ingroup module_ustk_core

 This class represents a 3D ultrasound RF volume.

  <h3>Example</h3>
  The following example shows how to build a RF3D ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImageRF3D.h>

    int main()
    {
      // Update settings
      unsigned int RFSampleNumber = 200;
      unsigned int lineNumber = 200;
      unsigned int frameNumber = 30;
      double probeRadius = 0.0006;
      double scanLinePitch = 0.0007;
      bool isProbeConvex = true;
      double motorRadius = 0.004;
      double framePitch = 0.06;
      usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;
      double axialResolution = 0.001;
      usImagePreScanSettings  imageSettings(probeRadius, scanLinePitch, isProbeConvex, axialResolution);
      usMotorSettings motorSettings(motorRadius,framePitch,motorType);
      usImage3D<unsigned char> I(RFSampleNumber, lineNumber, frameNumber);
      usImageRF3D<unsigned char> rf3d;
      rf3d.setData(I);
      rf3d.setImageSettings(imageSettings);
      rf3d.setMotorSettings(motorSettings);
    }

  \endcode

*/
template<class Type>
class usImageRF3D : public usImage3D<Type>, public usImagePreScanSettings, public usMotorSettings {
public:

  usImageRF3D();
  usImageRF3D(unsigned int RFSampleNumber, unsigned int lineNumber, unsigned int frameNumber,
              double probeRadius=0.0, double motorRadius=0.0, double scanLinePitch=0.0, double framePitch=0.0,
              bool isImageConvex = false, usMotorType motorType = usMotorSettings::LinearMotor, double axial_resolution = 0.0);
  usImageRF3D(usImage3D<Type> image3D, usImagePreScanSettings imageSettings, usMotorSettings motorSettings);
  usImageRF3D(usImage3D<Type> image3D);
  usImageRF3D(usImagePreScanSettings imageSettings);
  usImageRF3D(const usImageRF3D<Type> &other);
  virtual ~usImageRF3D();

  unsigned int getRFSampleNumber() const ;
  unsigned int getLineNumber() const ;
  unsigned int getFrameNumber() const ;

  usImageRF3D<Type>& operator=(const usImageRF3D<Type> &other);
  bool operator==(const usImageRF3D<Type> &other);

  void setData(const usImage3D<Type> &image);
};

/**
* Basic constructor.
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D() : usImage3D<Type>(), usImagePreScanSettings(), usMotorSettings()
{

}

/**
* Full initializing constructor.
* @param RFSampleNumber number of A-samples in a line.
* @param lineNumber number of lines.
* @param frameNumber number of frames.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isTransducerConvex Boolean to specify if the image is acquired by a convex probe(true) or by a linear probe (false).
* @param motorType usMotorType to specify if the image is acquired by a linear motor (LinearMotor),
* by a small angle rotation motor (TiltingMotor), or by a 360&deg; roatation motor (RotationalMotor).
* @param axial_resolution The distance (in meters) between 2 successive pixels acquired along a scanline.
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(unsigned int RFSampleNumber, unsigned int lineNumber, unsigned int frameNumber,
                            double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                            bool isTransducerConvex, usMotorSettings::usMotorType motorType, double axial_resolution)
  : usImage3D<Type>(RFSampleNumber, lineNumber, frameNumber),
    usImagePreScanSettings(probeRadius, scanLinePitch, isTransducerConvex, axial_resolution),
    usMotorSettings(motorRadius, framePitch, motorType)
{

}

/**
* Copy constructor from usImage3D and usImageSettings.
* @param image3D usImage3D to copy.
* @param imageSettings usImagePreScanSettings to copy.
* @param motorSettings usMotorSettings to copy.
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(usImage3D<Type> image3D, usImagePreScanSettings imageSettings, usMotorSettings motorSettings)
  : usImage3D<Type>(image3D), usImagePreScanSettings(imageSettings), usMotorSettings(motorSettings)
{

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(usImage3D<Type> image3D) : usImage3D<Type>(image3D)
{

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param imageSettings usImagePreScanSettings to copy
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(usImagePreScanSettings imageSettings) : usImagePreScanSettings(imageSettings)
{

}

/**
* Copy constructor.
* @param other usImageRF3D to copy
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(const usImageRF3D& other)
  : usImage3D<Type>(other), usImagePreScanSettings(other), usMotorSettings(other)
{

}

/**
* Destructor.
*/
template<class Type>
usImageRF3D<Type>::~usImageRF3D()
{

}

/**
* Copy operator.
*/
template<class Type>
usImageRF3D<Type>& usImageRF3D<Type>::operator=(const usImageRF3D<Type> &other)
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
bool usImageRF3D<Type>::operator==(const usImageRF3D<Type> &other)
{
  return(usImage3D<Type>::operator== (other) &&
         usImagePreScanSettings::operator ==(other));
}

/**
* Prints information in a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImageRF3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
    static_cast<const usImagePreScanSettings &>(other);
}

/**
* Gets the number of RF samples along a scanline.
*/
template<class Type>
unsigned int usImageRF3D<Type>::getRFSampleNumber() const {
  return usImage3D<Type>::getDimY();
}

/**
* Gets the number of scanlines used to acquire the volume.
*/
template<class Type>
unsigned int usImageRF3D<Type>::getLineNumber() const {
  return usImage3D<Type>::getDimX();
}

/**
* Prints information in a stream.
*/
template<class Type>
unsigned int usImageRF3D<Type>::getFrameNumber() const {
  return usImage3D<Type>::getDimZ();
}

/**
* Setter for image data.
* @param image The image to set.
*/
template<class Type>
void usImageRF3D<Type>::setData(const usImage3D<Type> &image)
{
  usImage3D<Type>::operator=(image);
}

#endif // US_IMAGE_RF_3D_H
