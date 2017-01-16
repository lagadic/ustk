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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
* @file usImagePreScan3D.h
* @brief 3D pre-scan ultrasound image.
*/

#ifndef __usImagePreScan3D_h_
#define __usImagePreScan3D_h_

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
      distance (in meters) between two consecutive B-Mode samples along a scan line. To set this value use
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

  The following figure summarize these settings and shows the structure of an usImagePreScan3D image:
  \image html img-usImagePreScan3D.png

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

  void insertFrame(vpImage<Type> frame, unsigned int index);

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
    throw(vpException(vpException::badValue, "3D pre-scan image X-size differ from transducer scan line number"));
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

/**
 * Insert at a given index to update the volume while grabbing successive 2D frames.
 * @param frame The 2D frame to insert.
 * @param index Position to insert the frame in the volume.
 */
template<class Type>
void usImagePreScan3D<Type>::insertFrame(vpImage<Type> frame, unsigned int index)
{
  //Dimentions checks
  if(index > this->getDimZ())
    throw(vpException(vpException::badValue,"usImage3D::insertFrame : frame index out of volume"));

  if(frame.getHeight() != this->getDimY() || frame.getWidth() != this->getDimX())
    throw(vpException(vpException::badValue,"usImage3D::insertFrame : frame size don't match volume size"));

  //offset to access the frame in the volume
  int offset = index * this->getDimY() * this->getDimX();
  Type* frameBeginning = this->getData() + offset;

  //copy
  for(unsigned int i=0; i<this->getDimX(); i++) {
    for(unsigned int j=0; j<this->getDimY(); j++) {
      frameBeginning[i + this->getDimX() * j] = frame[j][i];
    }
  }
}
#endif // US_IMAGE_PRESCAN_3D_H
