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

  This class represents a 2D RF ultrasound image. This image is nothing more than an image (respecting column major
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

  const Type *const getBitmap() const;

  unsigned int getHeight() const;
  unsigned int getNumberOfPixel() const;
  unsigned int getRFSampleNumber() const;
  unsigned int getWidth() const;

  //! Set the size of the image
  void init(unsigned int height, unsigned int width);

  usImageRF2D<Type> &operator=(const usImageRF2D<Type> &other);
  bool operator==(const usImageRF2D<Type> &other);

  //! operator[] allows operation like I[i] = x.
  inline Type *operator[](const unsigned int i) { return col[i]; }
  inline Type *operator[](const int i) { return col[i]; }

  //! operator[] allows operation like x = I[i]
  inline const Type *operator[](unsigned int i) const { return col[i]; }
  inline const Type *operator[](int i) const { return col[i]; }

  void setScanLineNumber(unsigned int scanLineNumber);

  void resize(const unsigned int h, const unsigned int w);
  void resize(const unsigned int h, const unsigned int w, const Type val);

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
* @param image 2D RF image.
*/
template <class Type> usImageRF2D<Type>::usImageRF2D(unsigned int height, unsigned int width) : usImagePreScanSettings()
{
  init(height, width);
}

/**
* Initializing constructor.
* @param image 2D RF image.
* @param preScanSettings Pre-scan image settings.
*/
template <class Type>
usImageRF2D<Type>::usImageRF2D(unsigned int height, unsigned int width, const usImagePreScanSettings &preScanSettings)
  : usImagePreScanSettings(preScanSettings)
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
  : usImagePreScanSettings(other), npixels(0), width(0), height(0)
{
  // allocation and resize
  resize(other.getHeight(), other.getWidth());

  // filling pixels values
  memcpy(bitmap, other.getBitmap(), (size_t)(height * width * sizeof(usImageRF2D<Type>)));
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
 * \param h Image height.
 * \param w Image width.
 */
template <class Type> void usImageRF2D<Type>::resize(const unsigned int h, const unsigned int w)
{
  this->init(h, w);
  this->setScanLineNumber(w);
}

/*!
  \brief Image initialization

  Allocate memory for an [h x w] image, using column major image convention.

  \param w : Image width.
  \param h : Image height.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template <class Type> void usImageRF2D<Type>::init(unsigned int h, unsigned int w)
{
  if (w != this->width) {
    if (col != NULL) {
      delete[] col;
      col = NULL;
    }
  }

  if ((h != this->height) || (w != this->width)) {
    if (bitmap != NULL) {
      delete[] bitmap;
      bitmap = NULL;
    }
  }
  this->width = w;
  this->height = h;

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

template <class Type> unsigned int usImageRF2D<Type>::getHeight() const { return height; }

template <class Type> unsigned int usImageRF2D<Type>::getNumberOfPixel() const { return npixels; }

template <class Type> unsigned int usImageRF2D<Type>::getWidth() const { return width; }

template <class Type> const Type *const usImageRF2D<Type>::getBitmap() const { return bitmap; }

#endif // US_IMAGE_RF_2D_H
