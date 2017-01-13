/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usImagePreScan2D.h
 * @brief 2D pre-scan ultrasound image.
 */

#ifndef __usImagePreScan2D_h_
#define __usImagePreScan2D_h_

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/**
  @class usImagePreScan2D
  @brief 2D pre-scan ultrasound image.
  @ingroup module_ustk_core

  This class represents a 2D pre-scan ultrasound image. This image is nothing more than a vpImage that
  contains ultrasound 2D pre-scan data and additional settings that give information about the
  acquisition process done by the transducer.

  The settings associated to an usImagePreScan2D image are the one implemented in usImagePreScanSettings.
  We recall that these settings are:
  - the transducer settings (see usTransducerSettings) that are:
    - the name of the probe that could be set using setProbeName() or retrieved using getProbeName().
    - the transducer radius \f$R_{_T}\f$ in meters (value set to zero for a linear transducer).
      Its value could be set using setTransducerRadius() and retrieved using getTransducerRadius().
    - the scan line pitch that corresponds to the angle \f$\alpha_{_{SC}}\f$ (in radians) between
      two successive scan line beams when the transducer is convex, or to the distance \f$d_{_{SC}}\f$
      (in meters) when the transducer is linear. To set this value use setScanLinePitch() and to get
      its value use getScanLinePitch().
    - the number of scan lines \f$n_{_{SC}}\f$. To set this setting use setScanLineNumber() and to access
      to the value use getScanLineNumber().
    - the type of ultrasound transducer used for data acquisition: convex or linear. This parameter
      could be set using setTransducerConvexity(). To know the transducer type use isTransducerConvex().
    - the depth that corresponds to the distance in meters between the first and the last pixel in a scan line.
      To set this value use setDepth() and to get the depth use getDepth().
    .
  - and an additional axial resolution parameter called \f$a_{_R}\f$ which corresponds to the
    distance (in meters) between two consecutive B-Mode samples along a scan line. To set this value use
    setAxialResolution() and to retrieve this value use getAxialResolution().

  The following figure summarize these settings and shows the structure of an usImagePreScan2D image:
  \image html img-usImagePreScan2D.png

  The following example shows how to build a 2D pre-scan ultrasound image from a vpImage and from acquisition settings.

  \code
#include <visp3/ustk_core/usImagePreScan2D.h>

int main()
{
  // 2D pre-scan image settings
  unsigned int BModeSampleNumber = 200;
  double transducerRadius = 0.007;
  double scanLinePitch = 0.04;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.005;

  vpImage<unsigned char> I(BModeSampleNumber, scanLineNumber);
  usImagePreScan2D<unsigned char> preScan2d;
  preScan2d.setTransducerRadius(transducerRadius);
  preScan2d.setScanLinePitch(scanLinePitch);
  preScan2d.setScanLineNumber(scanLineNumber);
  preScan2d.setTransducerConvexity(isTransducerConvex);
  preScan2d.setAxialResolution(axialResolution);
  preScan2d.setData(I);
}
  \endcode

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
  void setScanLineNumber(unsigned int scanLineNumber);

  //Filtering before calling vpImage::resize() to update scanLineNumber
  void resize(const unsigned int h, const unsigned int w);
  void resize(const unsigned int h, const unsigned int w, const Type val);

};

/**
* Basic constructor, all settings set to default values.
*/
template<class Type>
usImagePreScan2D<Type>::usImagePreScan2D()
  : vpImage<Type>(), usImagePreScanSettings()
{

}

/**
* Copy constructor.
* @param other 2D pre-scan image you want to copy.
*/
template<class Type>
usImagePreScan2D<Type>::usImagePreScan2D(const usImagePreScan2D &other) :
  vpImage<Type>(other), usImagePreScanSettings(other)
{

}

/**
* Constructor from an image and pre-scan settings.
* @param image Image you want to set.
* @param preScanSettings Pre-scan settings you want to set.
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
* Comparison operator.
*/
template<class Type>
bool usImagePreScan2D<Type>::operator==(const usImagePreScan2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) && usImagePreScanSettings::operator ==(other));
}

/**
* Operator to print 2D pre-scan image information on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePreScan2D<Type> &other)
{
  return out << static_cast<const usImagePreScanSettings &>(other)
             << "number of B-mode samples in a scan line : "
             << other.getBModeSampleNumber() << std::endl
             << "number of scan lines : "
             << other.getScanLineNumber() << std::endl;
}

/**
* Get the number of A-samples of B-Mode samples in a scan line.
* @return BModeSampleNumber Number of A-samples in a scan line.
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
  setScanLineNumber(image.getWidth());
}

/**
 * Set the transducer scan line number.
 *
 * Resize also the image width that is equal to the scan line number.
 * \param scanLineNumber Number of scan lines acquired by the transducer.
 */
template<class Type>
void usImagePreScan2D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  vpImage<Type>::resize(vpImage<Type>::getHeight(), scanLineNumber);
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

/*!
 * Resize the 2D pre-scan image.
 *
 * Updates also the transducer scan line number that corresponds to the image width.
 * \param h Image height.
 * \param w Image width.
 */
template<class Type>
void usImagePreScan2D<Type>::resize(const unsigned int h, const unsigned int w)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w);
}

/*!
 * Resize the 2D pre-scan image and set all the pixel to a given value.
 *
 * Updates also the transducer scan line number that corresponds to the image width.
 * \param h Image height.
 * \param w Image width.
 * \param val Value set to each pixel.
 */
template<class Type>
void usImagePreScan2D<Type>::resize(const unsigned int h, const unsigned int w, const Type val)
{
  usTransducerSettings::setScanLineNumber(w);
  vpImage<Type>::resize(h, w, val);
}

#endif // US_IMAGE_PRESCAN_2D_H
