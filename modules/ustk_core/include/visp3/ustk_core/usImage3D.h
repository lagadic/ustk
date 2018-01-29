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
  * Constructor. Set the dimensions of the volume.
  * @param dimU Volume width.
  * @param dimV Volume height.
  * @param dimW Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  */
  usImage3D(unsigned int dimU, unsigned int dimV, unsigned int dimW);

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
  * @param indexU Index on u-axis to acess
  * @param indexV Index on v-axis to acess
  * @param indexW Index on w-axis to acess
  * @return The pointer to the voxel specified indexes.
  */
  Type *getData(unsigned int indexU, unsigned int indexV, unsigned int indexW)
  {
    return framPointer[indexW] + m_dimU * indexV + indexU;
  }

  /**
  * Get the volume width.
  * @return The volume width, in number of voxels.
  */
  unsigned int getDimU() const { return m_dimU; }

  /**
  * Get the volume height.
  * @return The volume height, in number of voxels.
  */
  unsigned int getDimV() const { return m_dimV; }

  /**
  * Get the volume size along the w-axis (frame nubmer).
  * @return The w-axis size in voxels, in number of voxels.
  */
  unsigned int getDimW() const { return m_dimW; }

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
  * @param indexU Index on u-axis to acess
  * @param indexV Index on v-axis to acess
  * @param indexW Index on w-axis to acess
  */
  inline Type operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW) const
  {
    return framPointer[indexW][m_dimU * indexV + indexU];
  }

  /**
  * Modification operator.
  * @param indexU Index on u-axis to modify
  * @param indexV Index on v-axis to modify
  * @param indexW Index on w-axis to modify
  * @param value Value to insert at the desired index
  */
  inline void operator()(unsigned int indexU, unsigned int indexV, unsigned int indexW, Type value)
  {
    framPointer[indexW][m_dimU * indexV + indexU] = value;
  }

  /**
  * Resize the image if needed (if new dimensions differ from old ones).
  * @param dimU The volume size along u axis.
  * @param dimV The volume size along v axis.
  * @param dimW The volume size along w axis.
  */
  void resize(unsigned int dimU, unsigned int dimV, unsigned int dimW);

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
  * @param dimU Volume width (number of voxels).
  * @param dimV Volume height (number of voxels).
  * @param dimW Volume size (number of voxels) in the third dimension (orthogonal to ultrasound 2D frames).
  */
  void init(unsigned int dimU, unsigned int dimV, unsigned int dimW);

  unsigned int m_dimU; /**< Volume width in voxels (number of voxels on the u-axis)*/
  unsigned int m_dimV; /**< Volume height in voxels (number of voxels on the v-axis)*/
  unsigned int m_dimW; /**< Volume size in 3d dimension (number of voxels on the w-axis)*/
  unsigned int m_size; /**< Volume size : number of voxels in the whole volume*/

  Type *bitmap;       /**< Data container */
  Type **framPointer; /**< contains pointers on every first voxel of all frames in the volume */
};

/****************************************************************************
* Template implementations.
****************************************************************************/
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
template <class Type> inline void usImage3D<Type>::init(unsigned int dimU, unsigned int dimV, unsigned int dimW)
{
  if ((dimU != this->m_dimU) || (dimV != this->m_dimV) || (dimW != this->m_dimW)) {
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

  //  vpERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL) {
    vpERROR_TRACE("cannot allocate bitmap ");
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }
}

template <class Type>
usImage3D<Type>::usImage3D() : m_dimU(0), m_dimV(0), m_dimW(0), m_size(0), bitmap(NULL), framPointer(NULL)
{
}

template <class Type>
usImage3D<Type>::usImage3D(unsigned int dimU, unsigned int dimV, unsigned int dimW)
  : m_dimU(dimU), m_dimV(dimV), m_dimW(dimW), m_size(dimU * dimV * dimW), bitmap(NULL), framPointer(NULL)
{
  init(dimU, dimV, dimW);
  initData(0);
}

template <class Type> usImage3D<Type>::usImage3D(const usImage3D<Type> &volume, const bool copy)
{
  init(volume.getDimU(), volume.getDimV(), volume.getDimW());

  m_size = m_dimU * m_dimV * m_dimW;

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
  init(other.m_dimU, other.m_dimV, other.m_dimW);

  memcpy(bitmap, other.bitmap, m_size * sizeof(Type));
  return *this;
}

template <class Type> bool usImage3D<Type>::operator==(const usImage3D<Type> &other)
{
  bool settingsOk =
      this->getDimU() == other.getDimU() && this->getDimV() == other.getDimV() && this->getDimW() == other.getDimW();

  if (settingsOk) {
    for (unsigned int i = 0; i < m_size; i++) {
      if (bitmap[i] != other[i])
        return false;
    }
  } else
    return false;
  return true;
}

template <class Type> std::ostream &operator<<(std::ostream &out, const usImage3D<Type> &image)
{
  return out << "dim x: " << image.getDimU() << std::endl
             << "dim y: " << image.getDimV() << std::endl
             << "dim z: " << image.getDimW() << std::endl;
}

template <class Type> void usImage3D<Type>::setData(Type *data, int numberOfVoxels)
{
  try {
    m_size = numberOfVoxels;
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

template <class Type> void usImage3D<Type>::resize(unsigned int dimU, unsigned int dimV, unsigned int dimW)
{
  init(dimU, dimV, dimW);
}
#endif // US_IMAGE_3D_H
