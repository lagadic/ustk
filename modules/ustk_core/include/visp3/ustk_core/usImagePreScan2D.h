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
 *
 *****************************************************************************/

/**
 * @file usImagePreScan2D.h
 * @brief 2D pre-scan ultrasound image.
 */

#ifndef US_IMAGE_PRESCAN_2D_H
#define US_IMAGE_PRESCAN_2D_H

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/**
 @class usImagePreScan2D
 @brief 2D pre-scan ultrasound image.
 @ingroup module_ustk_core

  <h3>Example</h3>
  The following example shows how to build a 2D pre-scan ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImagePreScan2D.h>

    int main()
    {
      // Update settings
      unsigned int scanLineNumber = 200;
      unsigned int BModeSampleNumber = 200;
      double probeRadius = 0.06;
      double scanLinePitch = 0.04;
      bool isTransducerConvex = true;
      double axialResolution = 0.005;
      usImagePreScanSettings imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      vpImage<unsigned char> I(BModeSampleNumber, scanLineNumber);
      usImagePreScan2D<unsigned char> preScan2d;
      preScan2d.setData(I);
      preScan2d.setImageSettings(imageSettings);
    }
  \endcode

 This class represents a 2D ultrasound pre-scan frame.
 */
template<class Type>
class usImagePreScan2D : public vpImage<Type>, public usImagePreScanSettings {
public:
  //default constructors
  usImagePreScan2D();
  //All parameters initialisation constructors
  usImagePreScan2D(const vpImage<Type> &image, const usImagePreScanSettings &preScanSettings);
  //usImagePreScan2D copy constructor
  usImagePreScan2D(const usImagePreScan2D &other);

  //destructor
  virtual ~usImagePreScan2D();

  //Those parameters have to be passed in the constructor.
  unsigned int getBModeSampleNumber() const;

  //assignement
  usImagePreScan2D<Type>& operator=(const usImagePreScan2D<Type> &other);

  //comparison
  bool operator==(const usImagePreScan2D<Type> &other);

  void setData(const vpImage<Type> image);
  void setImageSettings(const usImagePreScanSettings &settings);
  void setScanLineNumber(unsigned int scanLineNumber);

  //Filtering before calling vpImage::resize() to update scanLineNumber
  void resize(const unsigned int h, const unsigned int w);
  void resize(const unsigned int h, const unsigned int w, const Type val);
};

/**
* Basic constructor, all settings set to default. For double data.
*/
template<class Type>
usImagePreScan2D<Type>::usImagePreScan2D()
  : vpImage<Type>(), usImagePreScanSettings()
{

}

/**
* Copy constructor.
* @param other usImagePreScan2D image you want to copy.
*/
template<class Type>
usImagePreScan2D<Type>::usImagePreScan2D(const usImagePreScan2D &other) :
  vpImage<Type>(other), usImagePreScanSettings(other)
{

}

/**
* Constructor from image and pre-scan settings.
* @param image Image you want to set.
* @param preScanSettings Image pre-scan settings you want to set.
*/
template<class Type>
usImagePreScan2D<Type>::usImagePreScan2D(const vpImage<Type> &image, const usImagePreScanSettings &preScanSettings) :
  vpImage<Type>(image), usImagePreScanSettings(preScanSettings)
{
  if (image.getWidth() != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "Pre-scan image width differ from transducer scanline number"));
}

/**
* Destructor.
*/
template<class Type>
usImagePreScan2D<Type>::~usImagePreScan2D() {}

/**
* Copy operator.
*/
template<class Type>
usImagePreScan2D<Type>& usImagePreScan2D<Type>::operator=(const usImagePreScan2D<Type> &other)
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
bool usImagePreScan2D<Type>::operator==(const usImagePreScan2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) && usImagePreScanSettings::operator ==(other));
}

/**
* Operator to print image informations on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePreScan2D<Type> &other)
{
  return out << static_cast<const usImagePreScanSettings &>(other)
             << "number of B-mode samples in a scanline : "
             << other.getBModeSampleNumber() << std::endl
             << "number of scanlines : "
             << other.getScanLineNumber() << std::endl;
}

/**
* Get the number of A-samples in a line.
* @return BModeSampleNumber Number of A-samples in a line.
*/
template<class Type>
unsigned int usImagePreScan2D<Type>::getBModeSampleNumber() const { return vpImage<Type>::getHeight(); }

/**
* Setter for the image data.
* @param image The image to set.
*/
template<class Type>
void usImagePreScan2D<Type>::setData(const vpImage<Type> image)
{
  vpImage<Type>::operator=(image);
  setScanLineNumber(image.getDimX());
}

/**
* Setter for the image settings.
* @param settings The new image settings.
*/
template<class Type>
void usImagePreScan2D<Type>::setImageSettings(const usImagePreScanSettings &settings)
{
  usImagePreScanSettings::operator=(settings);
}

/**
 * Set the scanline number that corresponds also to the image width.
 */
template<class Type>
void usImagePreScan2D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  vpImage<Type>::resize(vpImage<Type>::getHeight(), scanLineNumber);
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}


template<class Type>
void usImagePreScan2D<Type>::resize(const unsigned int h, const unsigned int w)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w);
}

template<class Type>
void usImagePreScan2D<Type>::resize(const unsigned int h, const unsigned int w, const Type val)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w, val);
}

#endif // US_IMAGE_PRESCAN_2D_H
