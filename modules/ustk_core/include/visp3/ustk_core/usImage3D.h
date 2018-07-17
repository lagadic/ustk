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
* @file usImage3D.h
* @brief 3D image handling.
*
* This class is used to represent 3D data.
*/

#ifndef __usImage3D_h_
#define __usImage3D_h_

#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>

/**
* @class usImage3D
* @brief Representation of a physical image volume.
* @ingroup module_ustk_core
*
* This class is used to represent 3D ultrasoubd data with physical information.
*/
template <class Type> class usImage3D
{
public:
  /**
  * Constructor.
  */
  usImage3D();

  /**
  * Constructor. Set the dimensions of the volume. Initialize the data container with default value.
  * @param height Volume height.
  * @param width Volume width.
  * @param frameNumber Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  */
  usImage3D(unsigned int height, unsigned int width, unsigned int frameNumber);

  /**
  * Constructor. Set the dimensions of the volume. Initialize the data container with the specified value.
  * @param height Volume height.
  * @param width Volume width.
  * @param frameNumber Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  * @param initialValue The initial data
  */
  usImage3D(unsigned int height, unsigned int width, unsigned int frameNumber, Type initialValue);

  /**
  * Copy constructor. By default performs a deep copy.
  * @param image other 3D-image to copy
  * @param copy boolean to select if deep copy is performed or not (deep copy by default)
  */
  usImage3D(const usImage3D<Type> &image, const bool copy = true);

  /**
  * Destructor.
  */
  virtual ~usImage3D();

  /** @name Inherited functionalities from usImage3D */
  //@{

  /**
  * Get the pointer to the const data container.
  * @return The pointer to the const data container.
  */
  Type *getConstData() const { return bitmap; }

  /**
  * Get the pointer to the data container.
  * @return The pointer to the data container.
  */
  Type *getData() { return bitmap; }

  /**
  * Get the pointer to the data container for specified position in the volume.
  * @param i Index on i-axis to acess
  * @param j Index on j-axis to acess
  * @param k Index on k-axis to acess
  * @return The pointer to the voxel specified indexes.
  */
  Type *getData(unsigned int i, unsigned int j, unsigned int k) { return framPointer[k] + m_width * i + j; }

  /**
  * Get the volume width.
  * @return The volume width, in number of voxels.
  */
  unsigned int getWidth() const { return m_width; }

  /**
  * Get the volume height.
  * @return The volume height, in number of voxels.
  */
  unsigned int getHeight() const { return m_height; }

  /**
  * Get the volume size along the k-axis (frame nubmer).
  * @return The k-axis size in voxels, in number of voxels.
  */
  unsigned int getNumberOfFrames() const { return m_numberOfFrames; }

  /**
  * Get the volume size.
  * @return The number of voxels in the volume.
  */
  unsigned int getSize() const { return m_size; }

  /**
  * Initialize the data container with the specified value.
  * @param value The data
  */
  void initData(Type value);

  /**
  * Assignment operator.
  * @param other other 3D-image to copy
  */
  usImage3D<Type> &operator=(const usImage3D<Type> &other);

  /**
  * Comparison operator.
  * @param other The 3d image to compare. Comparing image parameters, and all volume voxel by voxel.
  */
  bool operator==(const usImage3D<Type> &other);

  /**
  * Access operator.
  * @param i Index on i-axis to acess
  * @param j Index on j-axis to acess
  * @param k Index on k-axis to acess
  */
  inline Type operator()(unsigned int i, unsigned int j, unsigned int k) const
  {
    return framPointer[k][m_width * i + j];
  }

  /**
  * Modification operator.
  * @param i Index on i-axis to modify
  * @param j Index on j-axis to modify
  * @param k Index on k-axis to modify
  * @param value Value to insert at the desired index
  */
  inline void operator()(unsigned int i, unsigned int j, unsigned int k, Type value)
  {
    framPointer[k][m_width * i + j] = value;
  }

  /**
  * Resize the image if needed (if new dimensions differ from old ones).
  * @param height The volume size along i axis.
  * @param width The volume size along j axis.
  * @param numberOfFrames The volume size along k axis.
  */
  void resize(unsigned int height, unsigned int width, unsigned int numberOfFrames);

  /**
  * Set the data container.
  * @param data The data container.
  * @param numberOfVoxels The number of voxels in the image.
  */
  void setData(Type *data, int numberOfVoxels);

  //@}

protected:
private:
  /**
  * Initiation of the image.
  * @param height Volume height (number of voxels).
  * @param width Volume width (number of voxels).
  * @param numberOfFrames Volume size (number of voxels) in the third dimension (orthogonal to ultrasound 2D frames).
  */
  void init(unsigned int height, unsigned int width, unsigned int numberOfFrames);

  unsigned int m_width;          /**< Volume width in voxels (number of voxels on the j-axis)*/
  unsigned int m_height;         /**< Volume height in voxels (number of voxels on the i-axis)*/
  unsigned int m_numberOfFrames; /**< Volume size in 3d dimension (number of voxels on the k-axis)*/
  unsigned int m_size;           /**< Volume size : number of voxels in the whole volume*/

  Type *bitmap;       /**< Data container */
  Type **framPointer; /**< contains pointers on every first voxel of all frames in the volume */
};

/****************************************************************************
* Template implementations.
****************************************************************************/
/*!
  \brief Image initialization

  Allocate memory for an [width x height x numberOfFrames] image.
  \param height : Height of the 2D planes contained in the volume.
  \param width : Width of the 2D planes contained in the volume.
  \param numberOfFrames : Volume dimension in the 3rd dimension.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template <class Type>
inline void usImage3D<Type>::init(unsigned int height, unsigned int width, unsigned int numberOfFrames)
{
  if ((width != this->m_width) || (height != this->m_height) || (numberOfFrames != this->m_numberOfFrames)) {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10, "Destruction bitmap[]");
      delete[] bitmap;
      bitmap = NULL;
    }
    if (framPointer != NULL) {
      vpDEBUG_TRACE(10, "Destruction framPointer[]");
      delete[] framPointer;
      framPointer = NULL;
    }
  }

  m_width = width;
  m_height = height;
  m_numberOfFrames = numberOfFrames;

  m_size = m_width * m_height * m_numberOfFrames;

  if (bitmap == NULL)
    bitmap = new Type[m_size];

  if (framPointer == NULL)
    framPointer = new Type *[numberOfFrames];

  for (unsigned int k = 0; k < numberOfFrames; k++)
    framPointer[k] = bitmap + m_width * m_height * k;

  //  vpERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL) {
    vpERROR_TRACE("cannot allocate bitmap ");
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }
}

template <class Type>
usImage3D<Type>::usImage3D() : m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
}

template <class Type>
usImage3D<Type>::usImage3D(unsigned int height, unsigned int width, unsigned int frameNumber)
  : m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
  this->init(height, width, frameNumber);
  this->initData(Type());
}

template <class Type>
usImage3D<Type>::usImage3D(unsigned int height, unsigned int width, unsigned int frameNumber, Type initialValue)
  : m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
  this->init(height, width, frameNumber);
  this->initData(initialValue);
}

template <class Type>
usImage3D<Type>::usImage3D(const usImage3D<Type> &volume, const bool copy)
  : m_width(0), m_height(0), m_numberOfFrames(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
  this->init(volume.getHeight(), volume.getWidth(), volume.getNumberOfFrames());

  // deep copy
  if (copy)
    memcpy(bitmap, volume.bitmap, m_size * sizeof(Type));
}

template <class Type> usImage3D<Type>::~usImage3D()
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

template <class Type> usImage3D<Type> &usImage3D<Type>::operator=(const usImage3D<Type> &other)
{
  this->init(other.m_height, other.m_width, other.m_numberOfFrames);

  memcpy(bitmap, other.bitmap, m_size * sizeof(Type));
  return *this;
}

template <class Type> bool usImage3D<Type>::operator==(const usImage3D<Type> &other)
{
  bool settingsOk = this->getWidth() == other.getWidth() && this->getHeight() == other.getHeight() &&
                    this->getNumberOfFrames() == other.getNumberOfFrames();

  if (settingsOk) {
    for (unsigned int i = 0; i < m_size; i++) {
      if (bitmap[i] != other.getConstData()[i])
        return false;
    }
  } else
    return false;
  return true;
}

template <class Type> std::ostream &operator<<(std::ostream &out, const usImage3D<Type> &image)
{
  return out << "width : " << image.getWidth() << std::endl
             << "height : " << image.getHeight() << std::endl
             << "frames in volume : " << image.getNumberOfFrames() << std::endl;
}

template <class Type> void usImage3D<Type>::setData(Type *data, int numberOfVoxels)
{
  try {
    if (m_size != numberOfVoxels) {
      throw(vpException(vpException::fatalError, "usImage3D::setData() error, bitmap dimensions mismatch."));
    }
    memcpy(bitmap, data, m_size * sizeof(Type));
  } catch (std::exception e) {
    std::cout << "usImage3D::setData(), error when trying to copy the data :" << std::endl;
    std::cout << e.what() << std::endl;
  }
}

template <class Type> void usImage3D<Type>::initData(Type value)
{
  try {
    std::fill_n(bitmap, m_size, value);
  } catch (std::exception e) {
    std::cout << e.what() << std::endl;
  }
}

template <class Type> void usImage3D<Type>::resize(unsigned int height, unsigned int width, unsigned int numberOfFrames)
{
  this->init(height, width, numberOfFrames);
}
#endif // __usImage3D_h_
