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

  This class represents a 3D pre-scan ultrasound image. This image is nothing more than an usImage3D that
  contains 3D pre-scan data and additional settings that give information about the acquisition process.

  The settings associated to an usImagePreScan3D image are the:
  - pre-scan image settings implemented in usImagePreScanSettings that are:
    - the transducer radius \f$R_T\f$ in meters (value set to zero for a linear transducer)
    - the scan line pitch that corresponds to the angle \f$\alpha_{SC}\f$ (in radians) between
      to successive scan line beams when the transducer is convex, or to the distance \f$d_{SC}\f$
      (in meters) when the transducer is linear
    - the number of scan lines \f$n_{SC}\f$
    - the type of ultrasound transducer used for data acquisition: convex or linear.
    - an axial resolution parameter called \f$a_R\f$ that correspond to the distance in meter
      between 2 RF samples.
    .
  - the motor settings implemented in usMotorSettings that are:
    - the type of motor used to move the transducer: linear, tilting (small rotation) or rotationnal (360&deg; rotation).
    - the motor radius \f$R_M\f$ (value set to zero for a linear motor)
    - the frame pitch that corresponds to the angle \f$\alpha_F\f$ (in radians) between
      to successive frame acquisitions when the motor is convex, or to the distance \f$d_F\f$ (in meters)
      when the motor is linear.
    - the frame number \f$n_F\f$ that corresponds to the number of frames acquired by the probe to generate the 3D volume.
    .
  .

  The following example shows how to build a 3D pre-scan ultrasound image from an usImage3D, and from acquisiton settings.

  \code
#include <visp3/ustk_core/usImagePreScan3D.h>

int main()
{
  // Pre-scan image settings
  unsigned int BModeSampleNumber = 200;
  double transducerRadius = 0.008;
  double scanLinePitch = 0.004;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.001;

  // Motor settings
  double motorRadius = 0.004;
  double framePitch = 0.06;
  unsigned int frameNumber = 10;
  usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;

  usImagePreScanSettings imagePreScanSettings;
  imagePreScanSettings.setTransducerRadius(transducerRadius);
  imagePreScanSettings.setScanLinePitch(scanLinePitch);
  imagePreScanSettings.setScanLineNumber(scanLineNumber);
  imagePreScanSettings.setTransducerConvexity(isTransducerConvex);
  imagePreScanSettings.setAxialResolution(axialResolution);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(motorRadius);
  motorSettings.setFramePitch(framePitch);
  motorSettings.setFrameNumber(frameNumber);
  motorSettings.setMotorType(motorType);

  usImage3D<unsigned char> I(BModeSampleNumber, scanLineNumber, frameNumber);

  usImagePreScan3D<unsigned char> preScan3d;
  preScan3d.setData(I);
  preScan3d.setImagePreScanSettings(imagePreScanSettings);
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
  usImagePreScan3D(const usImage3D<Type> &image, const usImagePreScanSettings &preScanSettings, const usMotorSettings &motorSettings);
  //usImagePreScan3D copy constructor
  usImagePreScan3D(const usImagePreScan3D &other);

  //destructor
  virtual ~usImagePreScan3D();

  unsigned int getBModeSampleNumber() const;

  //assignement
  usImagePreScan3D<Type>& operator=(const usImagePreScan3D<Type> &other);
  //comparison
  bool operator==(const usImagePreScan3D<Type> &other);

  void setData(const usImage3D<Type> &image);
  void setFrameNumber(unsigned int frameNumber);
  void setScanLineNumber(unsigned int scanLineNumber);

  //Filtering before calling vpImage::resize() to update scanLineNumber
  void resize(unsigned int dimX,unsigned int dimY,unsigned int dimZ);
};

/**
* Basic constructor, all settings set to default values.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D()
  : usImage3D<Type>(), usImagePreScanSettings(), usMotorSettings()
{

}

/**
* Copy constructor.
* @param image 3D data you want to copy.
* @param preScanSettings Pre-scan settings you want to copy.
* @param motorSettings Motor settings you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImage3D<Type> &image,
                                         const usImagePreScanSettings &preScanSettings,
                                         const usMotorSettings &motorSettings)
  : usImage3D<Type>(image), usImagePreScanSettings(preScanSettings), usMotorSettings(motorSettings)
{
  if (image.getDimX() != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "3D pre-scan image X-size differ from transducer scanline number"));
  if (image.getDimZ() != motorSettings.getFrameNumber())
    throw(vpException(vpException::badValue, "3D pre-scan image Z-size differ from motor frame number"));
}

/**
* Copy constructor.
* @param other 3D pre-scan image you want to copy.
*/
template<class Type>
usImagePreScan3D<Type>::usImagePreScan3D(const usImagePreScan3D &other)
  : usImage3D<Type>(other), usImagePreScanSettings(other), usMotorSettings(other)
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

  //for motor settings
  usMotorSettings::operator =(other);

  return *this;
}

/**
* Comparison operator.
*/
template<class Type>
bool usImagePreScan3D<Type>::operator==(const usImagePreScan3D<Type> &other)
{
  return(usImage3D<Type>::operator== (other) &&
         usImagePreScanSettings::operator== (other) &&
         usMotorSettings::operator ==(other));
}

/**
* Operator to print 3D pre-scan image information on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePreScan3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
                static_cast<const usImagePreScanSettings &>(other) <<
                static_cast<const usMotorSettings &>(other);
}

/**
* Gets the number of B-mode samples along a scan line. This value corresponds to the 3D image Y-size.
*/
template<class Type>
unsigned int usImagePreScan3D<Type>::getBModeSampleNumber() const {
  return usImage3D<Type>::getDimY();
}

/**
* Setter for the 3D pre-scan image data.
* @param image Data to set.
*/
template<class Type>
void usImagePreScan3D<Type>::setData(const usImage3D<Type> &image)
{
  usImage3D<Type>::operator=(image);
  setScanLineNumber(image.getDimX());
  setFrameNumber(image.getDimZ());
}

/**
 * Set the transducer scan line number.
 *
 * Resize also the image X-size that is equal to the scan line number.
 * \param scanLineNumber Number of scan lines acquired by the transducer.
 */
template<class Type>
void usImagePreScan3D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  usImage3D<Type>::resize(scanLineNumber, usImage3D<Type>::getDimY(), usImage3D<Type>::getDimZ());
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

/**
 * Set the motor frame number.
 *
 * Resize also the image Z-size that is equal to the frame number.
 * \param frameNumber Number of frames in the 3D volume.
 */
template<class Type>
void usImagePreScan3D<Type>::setFrameNumber(unsigned int frameNumber)
{
  usImage3D<Type>::resize(usImage3D<Type>::getDimX(), usImage3D<Type>::getDimY(), frameNumber);
  usMotorSettings::setFrameNumber(frameNumber);
}

/*!
 * Resize the 3D pre-scan image.
 *
 * Updates also the transducer scan line number that corresponds to the image X-size and
 * the motor frame number that corresponds to the image Z-size.
 * \param dimX Image X-size.
 * \param dimY Image Y-size.
 * \param dimZ Image Z-size.
 */
template<class Type>
void usImagePreScan3D<Type>::resize(unsigned int dimX, unsigned int dimY, unsigned int dimZ)
{
  usImage3D<Type>::resize(dimX, dimY, dimZ);
  usMotorSettings::setFrameNumber(dimZ);
  usTransducerSettings::setScanLineNumber(dimX);
}

#endif // US_IMAGE_PRESCAN_3D_H
