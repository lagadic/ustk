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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
* @file usImageRF3D.h
* @brief 3D RF ultrasound image.
*/

#ifndef __usImageRF3D_h_
#define __usImageRF3D_h_

#include <cstring>

#include <visp3/ustk_core/usImageRF2D.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>
#include <visp3/ustk_core/usMotorSettings.h>

/*!
  @class usImageRF3D
  @brief 3D Radio Frequence (RF) ultrasound image.
  @ingroup module_ustk_core

  This class represents a 3D RF ultrasound image. This image is nothing more than voxel container that
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

  The following example shows how to build a 3D RF ultrasound image, and set the acquisiton settings.

  \code
#include <visp3/ustk_core/usImageIo.h>
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

  usImageRF3D<short> rf3d(scanLineNumber,RFSampleNumber,frameNumber);
  //images settings
  rf3d.setTransducerRadius(transducerRadius);
  rf3d.setScanLinePitch(scanLinePitch);
  rf3d.setScanLineNumber(scanLineNumber);
  rf3d.setTransducerConvexity(isTransducerConvex);
  rf3d.setAxialResolution(axialResolution);
  // motor settings
  rf3d.setMotorRadius(motorRadius);
  rf3d.setFramePitch(framePitch);
  rf3d.setFrameNumber(frameNumber);
  rf3d.setMotorType(motorType);

  //fill a value in the bitmap, at u=5, v=6, w=7
  rf3d(5,6,7,10);

}
  \endcode

*/
template <class Type> class usImageRF3D : public usImagePreScanSettings, public usMotorSettings
{
  friend class usRawFileParser;

public:
  usImageRF3D();
  usImageRF3D(unsigned int dimU, unsigned int dimV, unsigned int dimW);
  usImageRF3D(unsigned int dimU, unsigned int dimV, unsigned int dimZ, const usImagePreScanSettings &imageSettings,
              const usMotorSettings &motorSettings);
  usImageRF3D(const usImageRF3D<Type> &other);
  virtual ~usImageRF3D();

  void getFrame(usImageRF2D<Type> &image, unsigned int index) const;

  const Type *getConstData() const;

  unsigned int getDimU() const;
  unsigned int getDimV() const;
  unsigned int getDimW() const;

  unsigned int getRFSampleNumber() const;

  unsigned int getSize() const;

  void insertFrame(const usImageRF2D<short> &frame, unsigned int index);

  usImageRF3D<Type> &operator=(const usImageRF3D<Type> &other);
  bool operator==(const usImageRF3D<Type> &other);

  Type operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW) const;
  void operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW, Type value);

  void setFrameNumber(unsigned int frameNumber);
  void setScanLineNumber(unsigned int scanLineNumber);

  void resize(unsigned int dimU, unsigned int dimV, unsigned int dimW);

private:
  void init(unsigned int dimU, unsigned int dimV, unsigned int dimW);

  unsigned int m_dimU; /**< Volume width in pixels (number of pixels on the u-axis)*/
  unsigned int m_dimV; /**< Volume height in pixels (number of pixels on the v-axis)*/
  unsigned int m_dimW; /**< Volume size in 3d dimension (number of frames in the volume)*/
  unsigned int m_size; /**< Volume size : number of voxels in the whole volume*/

  Type *bitmap;       /**< Data container */
  Type **framPointer; /**< contains pointers on every first voxel of all frames in the volume */
};

/**
* Basic constructor.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D()
  : usImagePreScanSettings(), usMotorSettings(), m_dimU(0), m_dimV(0), m_dimW(0), m_size(0), bitmap(NULL),
    framPointer(NULL)
{
}

/**
* Basic constructor.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(unsigned int dimU, unsigned int dimV, unsigned int dimW)
  : usImagePreScanSettings(), usMotorSettings(), m_dimU(0), m_dimV(0), m_dimW(0), m_size(0), bitmap(NULL),
    framPointer(NULL)
{
  resize(dimU, dimV, dimW);
}

/**
* Full initializing constructor.
* @param image3D 3D image to copy that corresponds to the 3D RF image data.
* @param preScanSettings Pre-scan settings to copy.
* @param motorSettings Motor settings to copy.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(unsigned int dimU, unsigned int dimV, unsigned int dimW,
                               const usImagePreScanSettings &preScanSettings, const usMotorSettings &motorSettings)
  : usImagePreScanSettings(preScanSettings), usMotorSettings(motorSettings), m_dimU(0), m_dimV(0), m_dimW(0), m_size(0),
    bitmap(NULL), framPointer(NULL)
{
  if (dimU != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "3D RF image U-size differ from transducer scanline number"));
  if (dimW != motorSettings.getFrameNumber())
    throw(vpException(vpException::badValue, "3D RF image W-size differ from motor frame number"));

  init(dimU, dimV, dimW);
}

/**
* Copy constructor.
* @param other 3D RF image to copy.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(const usImageRF3D &other)
  : usImagePreScanSettings(other), usMotorSettings(other), m_dimU(0), m_dimV(0), m_dimW(0), m_size(0), bitmap(NULL)
{
  init(other.getDimU(), other.getDimV(), other.getDimW());
  memcpy(bitmap, other.getConstData(), (size_t)(m_dimU * m_dimV * m_dimW * sizeof(Type)));
}

/**
* Destructor.
*/
template <class Type> usImageRF3D<Type>::~usImageRF3D()
{
  if (bitmap) {
    delete[] bitmap;
    bitmap = NULL;
  }
  if (framPointer) {
    delete[] framPointer;
    framPointer = NULL;
  }
}

/**
* Copy operator.
* @param other 3D RF image to copy.
*/
template <class Type> usImageRF3D<Type> &usImageRF3D<Type>::operator=(const usImageRF3D<Type> &other)
{
  // allocation and resize
  resize(other.getDimU(), other.getDimV(), other.getDimW());

  // filling voxel values
  memcpy(bitmap, other.getConstData(), (size_t)(m_dimU * m_dimV * m_dimW * sizeof(Type)));

  // from usImageSettings
  usImagePreScanSettings::operator=(other);

  // from usMotorSettings
  usMotorSettings::operator=(other);

  return *this;
}

/**
* Comparison operator.
* @param other 3D RF image to compare with.
*/
template <class Type> bool usImageRF3D<Type>::operator==(const usImageRF3D<Type> &other)
{
  // test image settings
  if (!usImagePreScanSettings::operator==(other))
    return false;
  if (!usMotorSettings::operator==(other))
    return false;

  // test dimentions
  if (this->m_dimU != other.getDimU())
    return false;
  if (this->m_dimV != other.getDimV())
    return false;
  if (this->m_dimW != other.getDimW())
    return false;

  // test bitmap
  for (unsigned int i = 0; i < m_size; i++) {
    if (bitmap[i] != other.bitmap[i]) {
      return false;
    }
  }

  return true;
}

/**
* Operator to print 3D RF image information on a stream.
*/
template <class Type> std::ostream &operator<<(std::ostream &out, const usImageRF3D<Type> &image)
{
  return out << "dimU : " << image.getDimU() << "\ndimV : " << image.getDimV() << "\ndimW : " << image.getDimW()
             << std::endl
             << static_cast<const usImagePreScanSettings &>(image) << static_cast<const usMotorSettings &>(image);
}

/**
* Gets the number of RF samples along a scan line.
*/
template <class Type> unsigned int usImageRF3D<Type>::getRFSampleNumber() const { return getDimV(); }

/**
 * Set the transducer scan line number.
 *
 * Resize also the image U-size that is equal to the scan line number.
 * \param scanLineNumber Number of scan lines acquired by the transducer.
 */
template <class Type> void usImageRF3D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  if (scanLineNumber != m_dimU)
    resize(scanLineNumber, getDimV(), getDimW());
  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

/**
 * Set the motor frame number.
 *
 * Resize also the image W-size that is equal to the frame number.
 * \param frameNumber Number of frames in the 3D volume.
 */
template <class Type> void usImageRF3D<Type>::setFrameNumber(unsigned int frameNumber)
{
  if (frameNumber != m_dimW)
    resize(getDimU(), getDimV(), frameNumber);
  usMotorSettings::setFrameNumber(frameNumber);
}

/*!
 * Resize the 3D RF image.
 *
 * Updates also the transducer scan line number that corresponds to the image U-size and
 * the motor frame number that corresponds to the image W-size.
 * \param dimU Image U-size.
 * \param dimV Image V-size.
 * \param dimW Image W-size.
 */
template <class Type> void usImageRF3D<Type>::resize(unsigned int dimU, unsigned int dimV, unsigned int dimW)
{
  init(dimU, dimV, dimW);
  usMotorSettings::setFrameNumber(dimW);
  usTransducerSettings::setScanLineNumber(dimU);
}

/**
 * Insert at a given index to update the volume while grabbing successive 2D frames.
 * @param frame The 2D frame to insert.
 * @param index Position to insert the frame in the volume.
 */
template <class Type> void usImageRF3D<Type>::insertFrame(const usImageRF2D<short> &frame, unsigned int index)
{
  // Dimentions checks
  if (index > this->getDimW())
    throw(vpException(vpException::badValue, "usImageRF3D::insertFrame : frame index out of volume"));

  if (frame.getHeight() != this->getDimV() || frame.getWidth() != this->getDimU())
    throw(vpException(vpException::badValue, "usImageRF3D::insertFrame : frame size don't match volume size"));

  // offset to access the frame in the volume
  int offset = index * this->getDimV() * this->getDimU();
  Type *frameBeginning = bitmap + offset;

  // copy
  for (unsigned int i = 0; i < this->getDimV(); i++) {
    for (unsigned int j = 0; j < this->getDimU(); j++) {
      frameBeginning[i + this->getDimV() * j] = frame[j][i];
    }
  }
}

/**
 * Returns a 2D slice of the volume at a given index.
 * @param [out] image The 2D frame.
 * @param [in] index Position of the frame to extract in the volume.
 */
template <class Type> void usImageRF3D<Type>::getFrame(usImageRF2D<Type> &image, unsigned int index) const
{
  // Dimentions checks
  if (index > this->getDimW())
    throw(vpException(vpException::badValue, "usImageRF3D::getFrame : frame index out of volume"));

  image.resize(this->getDimV(), this->getDimU());

  // offset to access the frame beginning in the volume
  int offset = index * this->getDimV() * this->getDimU();
  const Type *frameBeginning = this->getConstData() + offset;

  // copy
  for (unsigned int i = 0; i < this->getDimV(); i++) {
    for (unsigned int j = 0; j < this->getDimU(); j++) {
      image[j][i] = frameBeginning[i + this->getDimV() * j];
    }
  }
}

/*!
  \brief Image initialization

  Allocate memory for an [dimU x dimV x dimW] image.
  \param dimU : Width of the 2D planes contained in the volume.
  \param dimV : Height of the 2D planes contained in the volume.
  \param dimW : Volume dimension in the 3rd dimension.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template <class Type> void usImageRF3D<Type>::init(unsigned int dimU, unsigned int dimV, unsigned int dimW)
{
  if ((dimU != this->m_dimU) || (dimV != this->m_dimV) || (dimW != this->m_dimW)) {
    if (bitmap != NULL) {
      delete[] bitmap;
      bitmap = NULL;
    }
    if (framPointer != NULL) {
      delete[] framPointer;
      framPointer = NULL;
    }
  }

  this->m_dimU = dimU;
  this->m_dimV = dimV;
  this->m_dimW = dimW;

  m_size = m_dimU * m_dimV * m_dimW;

  if (bitmap == NULL)
    bitmap = new Type[m_size];

  if (framPointer == NULL)
    framPointer = new Type *[dimW];

  for (unsigned int k = 0; k < dimW; k++)
    framPointer[k] = bitmap + m_dimU * m_dimV * k;

  if (bitmap == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }
}

/**
* Get the volume width.
* @return The volume width, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getDimU() const { return m_dimU; }

/**
* Get the volume height.
* @return The volume height, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getDimV() const { return m_dimV; }

/**
* Get the volume size along the w-axis.
* @return The w-axis size in voxels, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getDimW() const { return m_dimW; }

/**
* Get the volume size.
* @return The number of voxels in the volume.
*/
template <class Type> unsigned int usImageRF3D<Type>::getSize() const { return m_size; }

/**
* Get the bitmap pointer (read-only).
* @return The consted bitmap pointer.
*/
template <class Type> const Type *usImageRF3D<Type>::getConstData() const { return bitmap; }

/**
* @brief Access operator for value in voxel (indexU, indexV, indexW)
* @param indexU Index along u-axis to access (from 0 to dimU-1).
* @param indexV Index along v-axis to access (from 0 to dimV-1).
* @param indexW Index along w-axis to access (from 0 to dimW-1).
*/
template <class Type>
Type usImageRF3D<Type>::operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW) const
{
  bool indexOK = indexU < m_dimU && indexV < m_dimV && indexW < m_dimW;
  if (!indexOK)
    throw(vpException(vpException::dimensionError, "usImageRF3D : accessing a voxel out of range !"));

  return framPointer[indexW][m_dimV * indexU + indexV];
}

/**
* @brief Set value in voxel (indexU, indexV, indexW)
* @param indexU Index along u-axis to write in (from 0 to dimU-1).
* @param indexV Index along v-axis to write in (from 0 to dimV-1).
* @param indexW Index along w-axis to write in (from 0 to dimW-1).
* @param value Value to insert.
*/
template <class Type>
void usImageRF3D<Type>::operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW, Type value)
{
  bool indexOK = indexU < m_dimU && indexV < m_dimV && indexW < m_dimW;
  if (!indexOK)
    throw(vpException(vpException::dimensionError, "usImageRF3D : trying to write in a voxel out of range !"));
  framPointer[indexW][m_dimV * indexU + indexV] = value;
}

#endif // US_IMAGE_RF_3D_H
