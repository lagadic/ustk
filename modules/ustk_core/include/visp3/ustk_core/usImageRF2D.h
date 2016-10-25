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
 * @file usImageRF2D.h
 * @brief 2D RF ultrasound image.
 */

#ifndef US_IMAGE_RF_2D_H
#define US_IMAGE_RF_2D_H

#include <cstring>

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/*!
 @class usImageRF2D
 @brief 2D RF ultrasound image.
 @ingroup module_ustk_core

 This class represents a 2D ultrasound RF image. This image is nothing more than a vpImage that
  contains RF data and additional settings that give information about the acquisition process.

  <h3>Example</h3>
  The following example shows how to build a RF2D ultrasound image from a vpImage, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImageRF2D.h>

    int main()
    {
      // Update settings
      unsigned int scanLineNumber = 200;
      unsigned int RFSampleNumber = 200;
      double probeRadius = 0.007;
      double scanLinePitch = 0.0006;
      bool isTransducerConvex = true;
      double axialResolution = 0.002;
      usImagePreScanSettings imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      vpImage<unsigned char> I(RFSampleNumber, scanLineNumber);
      usImageRF2D<unsigned char> rf2d;
      rf2d.setData(I);
      rf2d.setImageSettings(imageSettings);
    }
  \endcode
 */
template<class Type>
class usImageRF2D : public vpImage<Type>, public usImagePreScanSettings {
public:
  
  usImageRF2D();
  usImageRF2D(const vpImage<Type> &image, const usImagePreScanSettings &preScanSettings);
  usImageRF2D(const usImageRF2D &other);
  virtual ~usImageRF2D();

  unsigned int getRFSampleNumber() const;

  usImageRF2D<Type>& operator=(const usImageRF2D<Type> &other);
  bool operator==(const usImageRF2D<Type> &other);

  void setData(const vpImage<Type> &image);
  void setScanLineNumber(unsigned int scanLineNumber);

  //Filtering before calling vpImage::resize() to update scanLineNumber
  void resize(const unsigned int h, const unsigned int w);
  void resize(const unsigned int h, const unsigned int w, const Type val);
};

/**
* Default constructor.
*/
template<class Type>
usImageRF2D<Type>::usImageRF2D()
  : vpImage<Type>(), usImagePreScanSettings()
{

}

/**
* Initializing constructor.
* @param image RF image.
* @param preScanSettings Pre-scan image settings.
*/
template<class Type>
usImageRF2D<Type>::usImageRF2D(const vpImage<Type> &image, const usImagePreScanSettings &preScanSettings)
  : vpImage<Type>(image), usImagePreScanSettings(preScanSettings)
{
  if (image.getWidth() != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "RF image width differ from transducer scanline number"));
}

/**
* Copy constructor.
* @param other usImageRF2D to copy
*/
template<class Type>
usImageRF2D<Type>::usImageRF2D(const usImageRF2D& other)
  : vpImage<Type>(other), usImagePreScanSettings(other)
{

}

/**
* Destructor.
*/
template<class Type>
usImageRF2D<Type>::~usImageRF2D()
{

}

/**
* Copy operator.
*/
template<class Type>
usImageRF2D<Type>& usImageRF2D<Type>::operator=(const usImageRF2D<Type> &other)
{
  //from vpImage
  vpImage<Type>::operator=(other);

  //from usImagePreScanSettings
  usImagePreScanSettings::operator=(other);

  return *this;
}

/**
* Comparaison operator.
*/
template<class Type>
bool usImageRF2D<Type>::operator==(const usImageRF2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) &&
         usImagePreScanSettings::operator ==(other));
}

/**
* Operator to print image informations on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImageRF2D<Type> &other)
{
  return out << static_cast<const usImagePreScanSettings &>(other) <<
    "number of A-samples in a scanline : " << other.getRFSampleNumber() << std::endl <<
    "number of scanlines : " << other.getScanLineNumber() << std::endl;
}

/**
* Get the number of RF-samples in a line.
* @return Number of RF-samples in a line.
*/
template<class Type>
unsigned int usImageRF2D<Type>::getRFSampleNumber() const { return vpImage<Type>::getHeight(); }

/**
* Setter for the image data and also the scan line number that corresponds to the image width.
* @param image The image to set.
*/
template<class Type>
void usImageRF2D<Type>::setData(const vpImage<Type> &image)
{
  vpImage<Type>::operator=(image);
  setScanLineNumber(image.getWidth());
}

/**
 * Set the scanline number that corresponds also to the image width.
 */
template<class Type>
void usImageRF2D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  vpImage<Type>::resize(vpImage<Type>::getHeight(), scanLineNumber);
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

template<class Type>
void usImageRF2D<Type>::resize(const unsigned int h, const unsigned int w)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w);
}

template<class Type>
void usImageRF2D<Type>::resize(const unsigned int h, const unsigned int w, const Type val)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w, val);
}

#endif // US_IMAGE_RF_2D_H
