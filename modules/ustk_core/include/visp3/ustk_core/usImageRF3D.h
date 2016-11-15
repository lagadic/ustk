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
  @brief 3D Radio Frequence (RF) ultrasound image.
  @ingroup module_ustk_core

  This class represents a 3D RF ultrasound image. This image is nothing more than an usImage3D that
  contains 3D RF data and additional settings that give information about the acquisition process.

  The settings associated to an usImageRF3D image are the:
  - pre-scan image settings implemented in usImagePreScanSettings that are:
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
    - an additional axial resolution parameter called \f$a_{_R}\f$ which corresponds to the
      distance (in meters) between two consecutive RF samples along a scan line. To set this value use
      setAxialResolution() and to retrieve this value use getAxialResolution().
    .
  - the motor settings implemented in usMotorSettings that are:
    - the type of motor used to move the transducer: linear, tilting (small rotation) or rotationnal
      (360&deg; rotation). This type is defined in usMotorType and could be set using setMotorType(). To retrieve
      the motor type use getMotorType().
    - the motor radius \f$R_{_M}\f$ (value set to zero for a linear motor). This value could be set using
      setMotorRadius() or get using getMotorRadius().
    - the frame pitch that corresponds to the angle \f$\alpha_{_F}\f$ (in radians) between
      to successive frame acquisitions when the motor is convex, or to the distance \f$d_{_F}\f$ (in meters)
      when the motor is linear. To set this value use setFramePitch() and to access use getFramePitch().
    - the frame number \f$n_{_F}\f$ that corresponds to the number of frames acquired by the probe to
      generate the 3D volume. This number is set using setFrameNumber() and could be retrieved using
      getFrameNumber().
    .
  .

  The following figure summarize these settings and shows the structure of an usImageRF3D image:
  \image html img-usImageRF3D.png

  The following example shows how to build a 3D RF ultrasound image from an usImage3D, and from acquisiton settings.

  \code
#include <visp3/ustk_core/usImageRF3D.h>

int main()
{
  // Pre-scan image settings
  unsigned int RFSampleNumber = 200;
  double transducerRadius = 0.0006;
  double scanLinePitch = 0.0007;
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

  usImage3D<unsigned char> I(RFSampleNumber, scanLineNumber, frameNumber);

  usImageRF3D<unsigned char> rf3d;
  rf3d.setData(I);
  rf3d.setImagePreScanSettings(imagePreScanSettings);
  rf3d.setMotorSettings(motorSettings);
}
  \endcode

*/
template<class Type>
class usImageRF3D : public usImage3D<Type>, public usImagePreScanSettings, public usMotorSettings
{
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
* @param image3D 3D image to copy that corresponds to the 3D RF image data.
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
* @param other 3D RF image to copy.
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
* Comparison operator.
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
* Operator to print 3D RF image information on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImageRF3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other) <<
                static_cast<const usImagePreScanSettings &>(other) <<
                static_cast<const usMotorSettings &>(other);
}

/**
* Gets the number of RF samples along a scan line.
*/
template<class Type>
unsigned int usImageRF3D<Type>::getRFSampleNumber() const {
  return usImage3D<Type>::getDimY();
}

/**
* Setter for 3D RF image data.
*
* Updates also the transducer scan line number that corresponds to the image X-size
* and the motor frame number that corresponds to the image Z-size.
* @param image The 3D data to set.
*/
template<class Type>
void usImageRF3D<Type>::setData(const usImage3D<Type> &image)
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
void usImageRF3D<Type>::setScanLineNumber(unsigned int scanLineNumber)
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
void usImageRF3D<Type>::setFrameNumber(unsigned int frameNumber)
{
  usImage3D<Type>::resize(usImage3D<Type>::getDimX(), usImage3D<Type>::getDimY(), frameNumber);
  usMotorSettings::setFrameNumber(frameNumber);
}

/*!
 * Resize the 3D RF image.
 *
 * Updates also the transducer scan line number that corresponds to the image X-size and
 * the motor frame number that corresponds to the image Z-size.
 * \param dimX Image X-size.
 * \param dimY Image Y-size.
 * \param dimZ Image Z-size.
 */
template<class Type>
void usImageRF3D<Type>::resize(unsigned int dimX, unsigned int dimY, unsigned int dimZ)
{
  usMotorSettings::setFrameNumber(dimZ);
  usTransducerSettings::setScanLineNumber(dimX);
  usImage3D<Type>::resize(dimX, dimY, dimZ);
}

#endif // US_IMAGE_RF_3D_H
