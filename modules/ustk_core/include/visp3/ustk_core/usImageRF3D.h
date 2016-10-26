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
      unsigned int scanLineNumber = 200;
      unsigned int frameNumber = 30;
      double probeRadius = 0.0006;
      double scanLinePitch = 0.0007;
      bool isTransducerConvex = true;
      double motorRadius = 0.004;
      double framePitch = 0.06;
      usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;
      double axialResolution = 0.001;
      usImagePreScanSettings  imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      usMotorSettings motorSettings(motorRadius,framePitch,motorType);
      usImage3D<unsigned char> I(RFSampleNumber, scanLineNumber, frameNumber);
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
  usImageRF3D(const usImage3D<Type> &image3D, const usImagePreScanSettings &imageSettings, const usMotorSettings &motorSettings);
  usImageRF3D(const usImageRF3D<Type> &other);
  virtual ~usImageRF3D();

  unsigned int getRFSampleNumber() const ;

  usImageRF3D<Type>& operator=(const usImageRF3D<Type> &other);
  bool operator==(const usImageRF3D<Type> &other);

  void setData(const usImage3D<Type> &image);
  void setFrameNumber(unsigned int frameNumber);
  void setScanLineNumber(unsigned int scanLineNumber);


  //Filtering before calling vpImage::resize() to update scanLineNumber
  void resize(unsigned int dimX,unsigned int dimY,unsigned int dimZ);
};

/**
* Basic constructor.
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D()
  : usImage3D<Type>(), usImagePreScanSettings(), usMotorSettings()
{

}

/**
* Full initializing constructor.
* @param image3D 3D Image to copy.
* @param preScanSettings Pre-scan settings to copy.
* @param motorSettings Motor settings to copy.
*/
template<class Type>
usImageRF3D<Type>::usImageRF3D(const usImage3D<Type> &image3D,
                               const usImagePreScanSettings &preScanSettings,
                               const usMotorSettings &motorSettings)
  : usImage3D<Type>(image3D), usImagePreScanSettings(preScanSettings), usMotorSettings(motorSettings)
{
  if (image3D.getDimX() != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "3D RF image X-size differ from transducer scanline number"));
  if (image3D.getDimZ() != motorSettings.getFrameNumber())
    throw(vpException(vpException::badValue, "3D RF image Z-size differ from motor frame number"));
}

/**
* Copy constructor.
* @param other 3D RF Image to copy.
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
* @param other 3D RF image to copy.
*/
template<class Type>
usImageRF3D<Type>& usImageRF3D<Type>::operator=(const usImageRF3D<Type> &other)
{
  //from vpImage
  usImage3D<Type>::operator=(other);

  //from usImageSettings
  usImagePreScanSettings::operator=(other);

  //from usMotorSettings
  usMotorSettings::operator=(other);

  return *this;
}

/**
* Comparaison operator.
* @param other 3D RF image to compare with.
*/
template<class Type>
bool usImageRF3D<Type>::operator==(const usImageRF3D<Type> &other)
{
  return(usImage3D<Type>::operator== (other) &&
         usImagePreScanSettings::operator ==(other) &&
         usMotorSettings::operator ==(other));
}

/**
* Prints information in a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImageRF3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
                static_cast<const usImagePreScanSettings &>(other) <<
                static_cast<const usMotorSettings &>(other);
}

/**
* Gets the number of RF samples along a scanline.
*/
template<class Type>
unsigned int usImageRF3D<Type>::getRFSampleNumber() const {
  return usImage3D<Type>::getDimY();
}

/**
* Setter for image data, and for scanline number and frame number.
* @param image The image to set.
*/
template<class Type>
void usImageRF3D<Type>::setData(const usImage3D<Type> &image)
{
  usImage3D<Type>::operator=(image);
  setScanLineNumber(image.getDimX());
  setFrameNumber(image.getDimZ());
}

/**
 * Set the scanline number that corresponds also to the 3D image X-dim size.
 */
template<class Type>
void usImageRF3D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  usImage3D<Type>::resize(scanLineNumber, usImage3D<Type>::getDimY(), usImage3D<Type>::getDimZ());
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

/**
* Setter for frame number.
* Setter for frame number that corresponds also to the 3D image Z-dim size.
*/
template<class Type>
void usImageRF3D<Type>::setFrameNumber(unsigned int frameNumber)
{
  usImage3D<Type>::resize(usImage3D<Type>::getDimX(), usImage3D<Type>::getDimY(), frameNumber);
  usMotorSettings::setFrameNumber(frameNumber);
}


template<class Type>
void usImageRF3D<Type>::resize(unsigned int dimX,unsigned int dimY,unsigned int dimZ)
{
  usMotorSettings::setFrameNumber(dimZ);
  usTransducerSettings::setScanLineNumber(dimX);
  usImage3D<Type>::resize(dimX, dimY, dimZ);
}

#endif // US_IMAGE_RF_3D_H
