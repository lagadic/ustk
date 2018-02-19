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
  usImageRF3D(unsigned int height, unsigned int width, unsigned int numberOfFrames);
  usImageRF3D(unsigned int height, unsigned int width, unsigned int numberOfFrames,
              const usImagePreScanSettings &imageSettings, const usMotorSettings &motorSettings);
  usImageRF3D(const usImageRF3D<Type> &other);
  virtual ~usImageRF3D();

  void getFrame(usImageRF2D<Type> &image, unsigned int index) const;

  const Type *getConstData() const;

  unsigned int getWidth() const;
  unsigned int getHeight() const;
  unsigned int getNumberOfFrames() const;

  unsigned int getRFSampleNumber() const;

  unsigned int getSize() const;

  void insertFrame(const usImageRF2D<short> &frame, unsigned int index);

  usImageRF3D<Type> &operator=(const usImageRF3D<Type> &other);
  bool operator==(const usImageRF3D<Type> &other);

  Type operator()(unsigned int i, unsigned int indexJ, unsigned int k) const;
  void operator()(unsigned int i, unsigned int indexJ, unsigned int k, const Type &value);

  void setFrameNumber(unsigned int frameNumber);
  void setScanLineNumber(unsigned int scanLineNumber);

  void resize(unsigned int height, unsigned int width, unsigned int numberOfFrames);

private:
  void init(unsigned int height, unsigned int width, unsigned int numberOfFrames);

  unsigned int m_width;          /**< Volume width in pixels (number of pixels on the j-axis)*/
  unsigned int m_height;         /**< Volume height in pixels (number of pixels on the v-axis)*/
  unsigned int m_numberOfFrames; /**< Volume size in 3d dimension (number of frames in the volume)*/
  unsigned int m_size;           /**< Volume size : number of voxels in the whole volume*/

  Type *bitmap;       /**< Data container */
  Type **framPointer; /**< contains pointers on every first voxel of all frames in the volume */
};

/**
* Basic constructor.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D()
  : usImagePreScanSettings(), usMotorSettings(), m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL),
    framPointer(NULL)
{
}

/**
* Basic constructor.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(unsigned int height, unsigned int width, unsigned int numberOfFrames)
  : usImagePreScanSettings(), usMotorSettings(), m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL),
    framPointer(NULL)
{
  resize(height, width, numberOfFrames);
}

/**
* Full initializing constructor.
* @param height Image height.
* @param width Image width.
* @param numberOfFrames Number of frames in volume.
* @param preScanSettings Pre-scan settings to copy.
* @param motorSettings Motor settings to copy.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(unsigned int height, unsigned int width, unsigned int numberOfFrames,
                               const usImagePreScanSettings &preScanSettings, const usMotorSettings &motorSettings)
  : usImagePreScanSettings(preScanSettings), usMotorSettings(motorSettings), m_width(0), m_height(0),
    m_numberOfFrames(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
  if (width != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "3D RF image U-size differ from transducer scanline number"));
  if (numberOfFrames != motorSettings.getFrameNumber())
    throw(vpException(vpException::badValue, "3D RF image W-size differ from motor frame number"));

  init(height, width, numberOfFrames);
}

/**
* Copy constructor.
* @param other 3D RF image to copy.
*/
template <class Type>
usImageRF3D<Type>::usImageRF3D(const usImageRF3D &other)
  : usImagePreScanSettings(other), usMotorSettings(other), m_width(0), m_height(0), m_numberOfFrames(0), m_size(0),
    bitmap(NULL)
{
  init(other.getWidth(), other.getHeight(), other.getNumberOfFrames());
  memcpy(bitmap, other.getConstData(), (size_t)(m_width * m_height * m_numberOfFrames * sizeof(Type)));
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
  resize(other.getWidth(), other.getHeight(), other.getNumberOfFrames());

  // filling voxel values
  memcpy(bitmap, other.getConstData(), (size_t)(m_width * m_height * m_numberOfFrames * sizeof(Type)));

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
  if (this->m_width != other.getWidth())
    return false;
  if (this->m_height != other.getHeight())
    return false;
  if (this->m_numberOfFrames != other.getNumberOfFrames())
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
  return out << "width : " << image.getWidth() << "\nheight : " << image.getHeight()
             << "\nnumberOfFrames : " << image.getNumberOfFrames() << std::endl
             << static_cast<const usImagePreScanSettings &>(image) << static_cast<const usMotorSettings &>(image);
}

/**
* Gets the number of RF samples along a scan line.
*/
template <class Type> unsigned int usImageRF3D<Type>::getRFSampleNumber() const { return getHeight(); }

/**
 * Set the transducer scan line number.
 *
 * Resize also the image U-size that is equal to the scan line number.
 * \param scanLineNumber Number of scan lines acquired by the transducer.
 */
template <class Type> void usImageRF3D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  if (scanLineNumber != m_width)
    resize(scanLineNumber, getHeight(), getNumberOfFrames());
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
  if (frameNumber != m_numberOfFrames)
    resize(getWidth(), getHeight(), frameNumber);
  usMotorSettings::setFrameNumber(frameNumber);
}

/*!
 * Resize the 3D RF image.
 *
 * Updates also the transducer scan line number that corresponds to the image U-size and
 * the motor frame number that corresponds to the image W-size.
 * \param height Image height.
 * \param width Image width.
 * \param numberOfFrames Number of frames in the volume.
 */
template <class Type>
void usImageRF3D<Type>::resize(unsigned int height, unsigned int width, unsigned int numberOfFrames)
{
  init(height, width, numberOfFrames);
  usMotorSettings::setFrameNumber(numberOfFrames);
  usTransducerSettings::setScanLineNumber(width);
}

/**
 * Insert at a given index to update the volume while grabbing successive 2D frames.
 * @param frame The 2D frame to insert.
 * @param index Position to insert the frame in the volume.
 */
template <class Type> void usImageRF3D<Type>::insertFrame(const usImageRF2D<short> &frame, unsigned int index)
{
  // Dimentions checks
  if (index > this->getNumberOfFrames())
    throw(vpException(vpException::badValue, "usImageRF3D::insertFrame : frame index out of volume"));

  if (frame.getHeight() != this->getHeight() || frame.getWidth() != this->getWidth())
    throw(vpException(vpException::badValue, "usImageRF3D::insertFrame : frame size don't match volume size"));

  // offset to access the frame in the volume
  int offset = index * this->getHeight() * this->getWidth();
  Type *frameBeginning = bitmap + offset;

  // copy
  for (unsigned int i = 0; i < this->getHeight(); i++) {
    for (unsigned int j = 0; j < this->getWidth(); j++) {
      frameBeginning[i + this->getHeight() * j] = frame(i, j);
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
  if (index > this->getNumberOfFrames())
    throw(vpException(vpException::badValue, "usImageRF3D::getFrame : frame index out of volume"));

  image.resize(this->getHeight(), this->getWidth());

  // offset to access the frame beginning in the volume
  int offset = index * this->getHeight() * this->getWidth();
  const Type *frameBeginning = this->getConstData() + offset;

  // copy
  for (unsigned int i = 0; i < this->getHeight(); i++) {
    for (unsigned int j = 0; j < this->getWidth(); j++) {
      image(i, j, frameBeginning[i + this->getHeight() * j]);
    }
  }
}

/*!
  \brief Image initialization

  Allocate memory for an [width x height x numberOfFrames] image.
  \param width : Width of the 2D planes contained in the volume.
  \param height : Height of the 2D planes contained in the volume.
  \param numberOfFrames : Volume dimension in the 3rd dimension.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template <class Type> void usImageRF3D<Type>::init(unsigned int height, unsigned int width, unsigned int numberOfFrames)
{
  if ((width != this->m_width) || (height != this->m_height) || (numberOfFrames != this->m_numberOfFrames)) {
    if (bitmap != NULL) {
      delete[] bitmap;
      bitmap = NULL;
    }
    if (framPointer != NULL) {
      delete[] framPointer;
      framPointer = NULL;
    }
  }

  this->m_width = width;
  this->m_height = height;
  this->m_numberOfFrames = numberOfFrames;

  m_size = m_width * m_height * m_numberOfFrames;

  if (bitmap == NULL)
    bitmap = new Type[m_size];

  if (framPointer == NULL)
    framPointer = new Type *[numberOfFrames];

  for (unsigned int k = 0; k < numberOfFrames; k++)
    framPointer[k] = bitmap + m_width * m_height * k;

  if (bitmap == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }
}

/**
* Get the volume width.
* @return The volume width, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getWidth() const { return m_width; }

/**
* Get the volume height.
* @return The volume height, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getHeight() const { return m_height; }

/**
* Get the volume size along the w-axis.
* @return The w-axis size in voxels, in number of voxels.
*/
template <class Type> unsigned int usImageRF3D<Type>::getNumberOfFrames() const { return m_numberOfFrames; }

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
* @brief Access operator for value in voxel (i, j, k)
* @param i Index along i-axis to access (from 0 to height-1).
* @param j Index along j-axis to access (from 0 to width-1).
* @param k Index along k-axis to access (from 0 to numberOfFrames-1).
*/
template <class Type> Type usImageRF3D<Type>::operator()(unsigned int i, unsigned int j, unsigned int k) const
{
  bool indexOK = j < m_width && i < m_height && k < m_numberOfFrames;
  if (!indexOK)
    throw(vpException(vpException::dimensionError, "usImageRF3D : accessing a voxel out of range !"));

  return framPointer[k][m_height * j + i];
}

/**
* @brief Set value in voxel (i, j, k)
* @param i Index along i-axis to write in (from 0 to height-1).
* @param j Index along j-axis to write in (from 0 to width-1).
* @param k Index along k-axis to write in (from 0 to numberOfFrames-1).
* @param value Value to insert.
*/
template <class Type>
void usImageRF3D<Type>::operator()(unsigned int i, unsigned int j, unsigned int k, const Type &value)
{
  bool indexOK = j < m_width && i < m_height && k < m_numberOfFrames;
  if (!indexOK)
    throw(vpException(vpException::dimensionError, "usImageRF3D : trying to write in a voxel out of range !"));
  framPointer[k][m_height * j + i] = value;
}

#endif // US_IMAGE_RF_3D_H
