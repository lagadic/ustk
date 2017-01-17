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
* @file usImagePostScan3D.h
* @brief 3D post-scan ultrasound image.
*/

#ifndef __usImagePostScan3D_h_
#define __usImagePostScan3D_h_

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usMotorSettings.h>

/**
  @class usImagePostScan3D
  @brief 3D post-scan ultrasound image.
  @ingroup module_ustk_core

  This class represents a 3D post-scan ultrasound image. This image is nothing more than an usImage3D that
  contains 3D post-scan data and additional settings that give information about the acquisition process.

  The settings associated to an usImagePostScan3D image are the:
  - transducer settings implemented in usTransducerSettings that are:
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

  The following figure summarize these settings and shows the structure of an usImagePostScan3D image when
  the motor is linear:
  \image html img-usImagePostScan3D-linear.png
  This other figure summarize these settings and shows the structure of an usImagePostScan3D image when
  the motor is convex:
  \image html img-usImagePostScan3D-convex.png

  The following example shows how to build a 3D post-scan ultrasound image from an usImage3D, and from acquisiton settings.
  \code
#include <visp3/ustk_core/usImagePostScan3D.h>

int main()
{
  // Transducer settings
  unsigned int BModeSampleNumber = 200;
  double transducerRadius = 0.045;;
  double scanLinePitch = 0.01;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.001;

  // Motor settings
  double motorRadius = 0.004;
  double framePitch = 0.06;
  unsigned int frameNumber = 10;
  usMotorSettings::usMotorType motorType = usMotorSettings::LinearMotor;

  // Post-scan data settings
  unsigned int dimX = 200;
  unsigned int dimY = 200;
  unsigned int dimZ = 20;

  usTransducerSettings transducerSettings;
  transducerSettings.setTransducerRadius(transducerRadius);
  transducerSettings.setScanLinePitch(scanLinePitch);
  transducerSettings.setScanLineNumber(scanLineNumber);
  transducerSettings.setTransducerConvexity(isTransducerConvex);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(motorRadius);
  motorSettings.setFramePitch(framePitch);
  motorSettings.setFrameNumber(frameNumber);
  motorSettings.setMotorType(motorType);

  usImage3D<unsigned char> I(dimX, dimY, dimZ);

  usImagePostScan3D<unsigned char> postScan3d;
  postScan3d.setData(I);
  postScan3d.setTransducerSettings(transducerSettings);
  postScan3d.setMotorSettings(motorSettings);
}
  \endcode

*/
template<class Type>
class usImagePostScan3D : public usImage3D<Type>, public usTransducerSettings, public usMotorSettings {
public:
  usImagePostScan3D();
  usImagePostScan3D(const usImage3D<Type> &image, const usTransducerSettings &transducerSettings,
                    const usMotorSettings &motorSettings,
                    double spacingX, double spacingY, double spacingZ, double scanLineDepth);
  usImagePostScan3D(const usImagePostScan3D &other);
  virtual ~usImagePostScan3D();

  double getElementSpacingX() const;
  double getElementSpacingY() const;
  double getElementSpacingZ() const;
  double getScanLineDepth() const;

  usImagePostScan3D<Type> & operator =(const usImagePostScan3D<Type> &other);

  bool operator ==(const usImagePostScan3D<Type> &other);

  void setData(const usImage3D<Type> &image3D);

  void setElementSpacingX(double elementSpacingX);
  void setElementSpacingY(double elementSpacingY);
  void setElementSpacingZ(double elementSpacingZ);
  void setScanLineDepth(double scanLineDepth);


private:
  double m_elementSpacingX; /**< Element spacing along the x-axis, in meters */
  double m_elementSpacingY; /**< Element spacing along the y-axis, in meters */
  double m_elementSpacingZ; /**< Element spacing along the z-axis, in meters */

  double m_scanLineDepth; /**< Distance between first and last pixel of a scan line, in meters */
};


/**
* Basic constructor, all parameters set to default values.
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D()
  : usImage3D<Type>(), usTransducerSettings(), usMotorSettings(),
    m_elementSpacingX(1.0), m_elementSpacingY(1.0), m_elementSpacingZ(1.0), m_scanLineDepth(0.0)
{

}

/**
* Copy constructor from an other 3D post-scan image.
* @param other 3D post-scan image to copy.
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D(const usImagePostScan3D &other)
  : usImage3D<Type>(other), usTransducerSettings(other), usMotorSettings(other),
    m_elementSpacingX(other.getElementSpacingX()), m_elementSpacingY(other.getElementSpacingY()),
    m_elementSpacingZ(other.getElementSpacingZ()), m_scanLineDepth(other.getScanLineDepth())
{

}

/**
* Constructor from 3D image data, transducer and motor settings.
* @param image 3D data to copy.
* @param transducerSettings Transducer settings to copy.
* @param motorSettings Motor settings to copy.
* @param spacingX distancee (in meters) between two voxels on X-axis
* @param spacingY distancee (in meters) between two voxels on Y-axis
* @param spacingZ distancee (in meters) between two voxels on Z-axis
* @param scanLineDepth distancee (in meters) between first and last voxel of a scan line.
*/
template<class Type>
usImagePostScan3D<Type>::usImagePostScan3D(const usImage3D<Type> &image,
                                           const usTransducerSettings &transducerSettings,
                                           const usMotorSettings &motorSettings,
                                           double spacingX, double spacingY, double spacingZ,
                                           double scanLineDepth)
: usImage3D<Type>(image), usTransducerSettings(transducerSettings), usMotorSettings(motorSettings),
  m_elementSpacingX (spacingX), m_elementSpacingY(spacingY), m_elementSpacingZ(spacingZ),
  m_scanLineDepth(scanLineDepth)
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

  //from settings
  usTransducerSettings::operator =(other);
  usMotorSettings::operator =(other);

  //from this class
  m_elementSpacingX = other.getElementSpacingX();
  m_elementSpacingY = other.getElementSpacingY();
  m_elementSpacingZ = other.getElementSpacingZ();
  m_scanLineDepth = other.getScanLineDepth();

  return *this;
}

/**
* Comparison operator.
*/
template<class Type>
bool usImagePostScan3D<Type>::operator == (usImagePostScan3D<Type> const& other)
{
  return usImage3D<Type>::operator ==(other) &&
         usTransducerSettings::operator==(other) &&
         usMotorSettings::operator ==(other) &&
         m_elementSpacingX == other.getElementSpacingX() &&
         m_elementSpacingY == other.getElementSpacingY() &&
         m_elementSpacingZ == other.getElementSpacingZ() &&
         m_scanLineDepth == other.getScanLineDepth();
}

/**
* Operator to print 3D post-scan image information on a stream.
*/
template<class Type> std::ostream& operator<<(std::ostream& out, const usImagePostScan3D<Type> &other)
{
  return out << static_cast<const usImage3D<Type> &>(other)
             << static_cast<const usTransducerSettings &>(other)
             << static_cast<const usMotorSettings &>(other)
             << "spacingX = " << other.getElementSpacingX() << std::endl
             << "spacingY = " << other.getElementSpacingY() << std::endl
             << "spacingZ = " << other.getElementSpacingZ() << std::endl
             << "scan line depth = " << other.getScanLineDepth() << std::endl;
}

/**
* Setter for image data.
* @param image3D 3D image data you want to set.
*/
template<class Type>
void usImagePostScan3D<Type>::setData(const usImage3D<Type> &image3D)
{
  usImage3D<Type>::operator =(image3D);
}

/**
* Get the element spacing along the x-axis.
* @return The element spacing along the x-axis, in meters.
*/
template<class Type>
double usImagePostScan3D<Type>::getElementSpacingX() const
{
  return m_elementSpacingX;
}

/**
* Get the element spacing along the y-axis.
* @return The element spacing along the y-axis, in meters.
*/
template<class Type>
double usImagePostScan3D<Type>::getElementSpacingY() const
{
  return m_elementSpacingY;
}

/**
* Get the element spacing along the z-axis.
* @return The element spacing along the z-axis, in meters.
*/
template<class Type>
double usImagePostScan3D<Type>::getElementSpacingZ() const
{
  return m_elementSpacingZ;
}


/**
* Get the scan line depth in meters.
* @return The scan line depth, in meters.
*/
template<class Type>
double usImagePostScan3D<Type>::getScanLineDepth() const
{
  return m_scanLineDepth;
}

/**
* Set the element spacing along the x-axis.
* @param elementSpacingX The element spacing along the x-axis, in meters.
*/
template<class Type>
void usImagePostScan3D<Type>::setElementSpacingX(double elementSpacingX)
{
  m_elementSpacingX = elementSpacingX;
}

/**
* Set the element spacing along the y-axis.
* @param elementSpacingY The element spacing along the y-axis, in meters.
*/
template<class Type>
void usImagePostScan3D<Type>::setElementSpacingY(double elementSpacingY)
{
  m_elementSpacingY = elementSpacingY;
}

/**
* Set the element spacing along the z-axis.
* @param elementSpacingZ The element spacing along the z-axis, in meters.
*/
template<class Type>
void usImagePostScan3D<Type>::setElementSpacingZ(double elementSpacingZ)
{
  m_elementSpacingZ = elementSpacingZ;
}

/**
* Set the scan line depth (distance between first and last pixel of a scan line).
* @param scanLineDepth The scan line depth, in meters.
*/
template<class Type>
void usImagePostScan3D<Type>::setScanLineDepth(double scanLineDepth)
{
  m_scanLineDepth = scanLineDepth;
}

#endif // US_IMAGE_POSTSCAN_3D_H
