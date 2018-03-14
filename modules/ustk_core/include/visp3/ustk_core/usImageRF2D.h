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
 * @file usImageRF2D.h
 * @brief 2D RF ultrasound image.
 */

#ifndef __usImageRF2D_h_
#define __usImageRF2D_h_

#include <cstring>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/*!
  @class usImageRF2D
  @brief 2D Radio Frequence (RF) ultrasound image.
  @ingroup module_ustk_core

  This class represents a 2D RF ultrasound image. This image is nothing more than an image (respecting column-major
bitmap storage) that  contains additional settings that give information about the acquisition process done by the
transducer.

  The settings associated to an usImageRF2D image are the one implemented in usImagePreScanSettings.
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
    distance (in meters) between two consecutive RF samples along a scan line. To set this value use
    setAxialResolution() and to retrieve this value use getAxialResolution().

  The following figure summarize these settings and shows the structure of an usImageRF2D image:
  \image html img-usImageRF2D.png

  The following example shows how to read a 2D RF ultrasound image, and set the image settings of your choice.

  \code
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_io/usImageIo.h>

int main()
{
  // 2D RF image settings
  unsigned int RFSampleNumber = 2000;
  double transducerRadius = 0.007;
  double scanLinePitch = 0.0006;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;

  usImageRF2D<short> rf2d(RFSampleNumber, scanLineNumber);

// sets the RF samples values contained in the .raw file in the bitmap of
//the usImageRF2D, and the settings contained in the mhd header.
usImageIo::read(rf2d, "path/to/file.mhd");

//sets your own settings
rf2d.setTransducerRadius(transducerRadius);
rf2d.setScanLinePitch(scanLinePitch);
rf2d.setScanLineNumber(scanLineNumber);
rf2d.setTransducerConvexity(isTransducerConvex);
rf2d.setAxialResolution(axialResolution);

return 0;
}
\endcode */

template <class Type> class usImageRF2D : public usImagePreScanSettings
{
  friend class usRawFileParser;
  friend class usNetworkGrabberRF2D;
  friend class usNetworkGrabberRF3D;
  friend class usVirtualServer;

public:
  usImageRF2D();
  usImageRF2D(unsigned int height, unsigned int width);
  usImageRF2D(unsigned int height, unsigned int width, const usImagePreScanSettings &preScanSettings);
  usImageRF2D(const usImageRF2D &other);
  virtual ~usImageRF2D();

  const Type *getBitmap() const;

  unsigned int getHeight() const;
  unsigned int getNumberOfPixel() const;
  unsigned int getRFSampleNumber() const;

  const Type *getSignal(unsigned int scanlineIndex) const;

  unsigned int getWidth() const;

  //! Set the size of the image
  void init(unsigned int height, unsigned int width);

  usImageRF2D<Type> &operator=(const usImageRF2D<Type> &other);
  bool operator==(const usImageRF2D<Type> &other);

  //! operator() allows to access/modify RF samples values in the image.
  Type operator()(unsigned int i, unsigned int j) const;
  void operator()(unsigned int i, unsigned int j, const Type &value);

  void setScanLineNumber(unsigned int scanLineNumber);

  void resize(const unsigned int height, const unsigned int width);
  void resize(const unsigned int height, const unsigned int width, const Type &val);

private:
  Type *bitmap;
  unsigned int npixels;
  unsigned int width;
  unsigned int height;
  Type **col;
};

/*!
  \brief Constructor

  No memory allocation is done
*/
template <class Type>
usImageRF2D<Type>::usImageRF2D() : usImagePreScanSettings(), bitmap(NULL), npixels(0), width(0), height(0), col(NULL)
{
}

/**
* Initializing constructor.
* @param height Image height.
* @param width Image width.
*/
template <class Type>
usImageRF2D<Type>::usImageRF2D(unsigned int height, unsigned int width)
  : usImagePreScanSettings(), bitmap(NULL), npixels(0), width(0), height(0), col(NULL)
{
  init(height, width);
}

/**
* Initializing constructor.
* @param height Image height.
* @param width Image width.
* @param preScanSettings Pre-scan image settings.
*/
template <class Type>
usImageRF2D<Type>::usImageRF2D(unsigned int height, unsigned int width, const usImagePreScanSettings &preScanSettings)
  : usImagePreScanSettings(preScanSettings), bitmap(NULL), npixels(0), width(0), height(0), col(NULL)
{
  if (width != preScanSettings.getScanLineNumber())
    throw(vpException(vpException::badValue, "RF image width differ from transducer scan line number"));

  init(height, width);
  setImagePreScanSettings(preScanSettings);
}

/**
* Copy constructor.
* @param other 2D RF image to copy
*/
template <class Type>
usImageRF2D<Type>::usImageRF2D(const usImageRF2D &other)
  : usImagePreScanSettings(other), bitmap(NULL), npixels(0), width(0), height(0), col(NULL)
{
  // allocation and resize
  resize(other.getHeight(), other.getWidth());

  // filling pixels values
  memcpy(bitmap, other.getBitmap(), (size_t)(height * width * sizeof(Type)));
}

/**
* Destructor.
*/
template <class Type> usImageRF2D<Type>::~usImageRF2D()
{
  if (bitmap != NULL) {
    delete[] bitmap;
    bitmap = NULL;
  }

  if (col != NULL) {
    delete[] col;
    col = NULL;
  }
}

/**
* Copy operator.
*/
template <class Type> usImageRF2D<Type> &usImageRF2D<Type>::operator=(const usImageRF2D<Type> &other)
{
  resize(other.getHeight(), other.getWidth());
  memcpy(bitmap, other.getBitmap(), height * width * sizeof(Type));

  // from usImagePreScanSettings
  usImagePreScanSettings::operator=(other);

  return *this;
}

/**
* Comparison operator.
*/
template <class Type> bool usImageRF2D<Type>::operator==(const usImageRF2D<Type> &other)
{

  if (this->width != other.getWidth())
    return false;
  if (this->height != other.getHeight())
    return false;

  for (unsigned int i = 0; i < npixels; i++) {
    if (bitmap[i] != other.bitmap[i]) {
      return false;
    }
  }
  return usImagePreScanSettings::operator==(other);
}

/**
* Access operator.
* @param i Row index of the pixel to access.
* @param j Column index of the pixel to access.
* @return The value of the pixel.
*/
template <class Type> Type usImageRF2D<Type>::operator()(unsigned int i, unsigned int j) const
{
  if (i >= height || j >= width)
    throw vpException(vpException::dimensionError, "usImageRF2D, try to acess index out of image bounds");

  return col[j][i];
}

/**
* Pixel writing operator.
* @param i Row index of the pixel to write.
* @param j Column index of the pixel to write.
* @param value The value to write.
*/
template <class Type> void usImageRF2D<Type>::operator()(unsigned int i, unsigned int j, const Type &value)
{
  if (i >= height || j >= width)
    throw vpException(vpException::dimensionError, "usImageRF2D, try to write at index out of image bounds");

  col[j][i] = value;
}

/**
* Operator to print 2D RF image information on a stream.
*/
template <class Type> std::ostream &operator<<(std::ostream &out, const usImageRF2D<Type> &other)
{
  return out << static_cast<const usImagePreScanSettings &>(other) << "image height : " << other.getHeight()
             << std::endl
             << "image width : " << other.getWidth() << std::endl
             << "number of A-samples in a scan line : " << other.getRFSampleNumber() << std::endl
             << "number of scan lines : " << other.getScanLineNumber() << std::endl;
}

/**
* Get the number of RF samples in a scan line.
* @return Number of RF samples in a scan line.
*/
template <class Type> unsigned int usImageRF2D<Type>::getRFSampleNumber() const { return getHeight(); }

/**
 * Set the transducer scan line number.
 *
 * Resize also the image width that is equal to the scan line number.
 * \param scanLineNumber Number of scan lines acquired by the transducer.
 */
template <class Type> void usImageRF2D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  if (scanLineNumber != getWidth())
    resize(this->getHeight(), scanLineNumber);

  usTransducerSettings::setScanLineNumber(scanLineNumber);
}

/*!
 * Resize the 2D RF image.
 *
 * Updates also the transducer scan line number that corresponds to the image width.
 * \param height Image height.
 * \param width Image width.
 */
template <class Type> void usImageRF2D<Type>::resize(const unsigned int height, const unsigned int width)
{
  this->init(height, width);
  this->setScanLineNumber(width);
}

/*!
 * Resize the 2D RF image.
 *
 * Updates also the transducer scan line number that corresponds to the image width.
 * \param height Image height.
 * \param width Image width.
 * \param val Value to set in every pixel.
 */
template <class Type>
void usImageRF2D<Type>::resize(const unsigned int height, const unsigned int width, const Type &val)
{
  this->init(height, width);
  this->setScanLineNumber(width);

  // fill bitmap
  for (unsigned int n = 0; n < this->npixels; n++) {
    bitmap[n] = val;
  }
}

/*!
  \brief Image initialization

  Allocate memory for an [h x w] image, using column major image convention.

  \param width : Image width.
  \param height : Image height.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template <class Type> void usImageRF2D<Type>::init(unsigned int height, unsigned int width)
{
  if (width != this->width) {
    if (col != NULL) {
      delete[] col;
      col = NULL;
    }
  }

  if ((height != this->height) || (width != this->width)) {
    if (bitmap != NULL) {
      delete[] bitmap;
      bitmap = NULL;
    }
  }
  this->width = width;
  this->height = height;

  npixels = width * height;
  if (bitmap == NULL)
    bitmap = new Type[npixels];

  if (bitmap == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }

  if (col == NULL)
    col = new Type *[width];
  if (col == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate col "));
  }

  unsigned int j;
  for (j = 0; j < width; j++)
    col[j] = bitmap + j * height;
}

/*!
 * Getter for the height of the image
 * \return Image height.
 */
template <class Type> unsigned int usImageRF2D<Type>::getHeight() const { return height; }

/*!
 * Getter for the number of pixels in the image.
 * \return Total count of pixels in the image.
 */
template <class Type> unsigned int usImageRF2D<Type>::getNumberOfPixel() const { return npixels; }

/*!
 * Getter for the width of the image
 * \return Image width.
 */
template <class Type> unsigned int usImageRF2D<Type>::getWidth() const { return width; }

/*!
 * Getter for a const pointer on the image bitmap.
 * \return Pointer on image bitmap.
 */
template <class Type> const Type *usImageRF2D<Type>::getBitmap() const { return bitmap; }

/*!
 * Getter for the RF signal at a certain scanline index.
 * \param scanlineIndex The index of the scanline to acess.
 * \return The RF signal.
 */
template <class Type> const Type *usImageRF2D<Type>::getSignal(unsigned int scanlineIndex) const
{
  return col[scanlineIndex];
}
#endif // US_IMAGE_RF_2D_H
