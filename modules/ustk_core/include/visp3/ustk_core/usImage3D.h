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
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*
* Authors:
* Pierre Chatelain
* Marc Pouliquen
*
*****************************************************************************/

/**
* @file usImage3D.h
* @brief 3D image handling.
*/

#ifndef US_IMAGE_3D_H
#define US_IMAGE_3D_H

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

/*
* Storage element type for MetaImage (mhd) format.
*/
/*
enum ElementType
{
  MET_UNKNOWN, MET_CHAR, MET_UCHAR, MET_SHORT, MET_USHORT, MET_LONG, MET_ULONG, MET_INT, MET_UINT,
  MET_FLOAT, MET_DOUBLE, MET_VECTOR
};*/

/*
* @class usImage3D
* @brief Representation of a physical image volume.
*
* This class is used to represent 3D data with physical information such as element spacing.
*/
template <class Type>
class usImage3D
{
public:
  /**
  * Constructor.
  */
  usImage3D();

  /**
  * Constructor. Set the dimensions and element spacing of the volume.
  */
  usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ, float spacingX, float spacingY, float spacingZ);

  /**
  * Copy constructor. By default performs a deep copy.
  */
  usImage3D(const usImage3D<Type> &image, const bool copy = false);

  /**
  * Destructor.
  */
  ~usImage3D();

  /**
  * Assignment operator.
  */
  usImage3D<Type> &operator=(const usImage3D<Type> &other);

  /**
  * Access operator.
  */
  inline Type operator()(unsigned int index) const
  {
    return m_data[index];
  }

  /**
  * Modification operator.
  */
  inline void operator()(unsigned int index, Type value)
  {
    m_data[index] = value;
  }

  /**
  * Access operator.
  */
  inline Type operator()(unsigned int x, unsigned int y, unsigned int z) const
  {
    return m_data[(m_dimx * m_dimy) * z + m_dimx*y + x];
  }

  /**
  * Modification operator.
  */
  inline void operator()(unsigned int x, unsigned int y, unsigned int z, Type value)
  {
    m_data[(m_dimx * m_dimy)*z + m_dimx*y + x] = value;
  }

  /**
  * Get the volume size.
  */
  unsigned int getSize() const { return m_size; }

  /**
  * Get the volume height.
  */
  unsigned int getDimX() const { return m_dimx; }

  /**
  * Get the volume width.
  */
  unsigned int getDimY() const { return m_dimy; }

  /**
  * Get the volume depth.
  */
  unsigned int getDimZ() const { return m_dimz; }

  /**
  * Get the element spacing along the x-axis.
  */
  float getElementSpacingX() const { return m_elementSpacingX; }

  /**
  * Get the element spacing along the y-axis.
  */
  float getElementSpacingY() const { return m_elementSpacingY; }

  /**
  * Get the element spacing along the y-axis.
  */
  float getElementSpacingZ() const { return m_elementSpacingZ; }

  /**
  * Set the element spacing along the x-axis.
  */
  void setElementSpacingX(float elementSpacingX) { m_elementSpacingX = elementSpacingX; }

  /**
  * Set the element spacing along the y-axis.
  */
  void setElementSpacingY(float elementSpacingY) { m_elementSpacingY = elementSpacingY; }

  /**
  * Set the element spacing along the y-axis.
  */
  void setElementSpacingZ(float elementSpacingZ) { m_elementSpacingZ = elementSpacingZ; }

  /**
  * Get the pointer to the data container.
  */
  Type* getData() { return m_data; }

  /**
  * Get the pointer to the const data container.
  */
  const Type* getConstData() const { return m_data; }

  /**
  * Set the data container.
  */
  void setData(Type* data);

  /**
  * Initialize the data container with the specified value.
  */
  void initData(Type value = Type());

protected:
  unsigned int m_dimx; /**< Volume height in pixels (number of pixels on the x-axis)*/
  unsigned int m_dimy; /**< Volume width in pixels (number of pixels on the x-axis)*/
  unsigned int m_dimz; /**< Volume depth in pixels (number of pixels on the x-axis)*/
  unsigned int m_size; /**< Volume size : number of voxels in the whole volume*/
  Type* m_data; /**< Data container */
  float m_elementSpacingX; /**< Element spacing along the x-axis, in meters */
  float m_elementSpacingY; /**< Element spacing along the y-axis, in meters */
  float m_elementSpacingZ; /**< Element spacing along the z-axis, in meters */
};

/****************************************************************************
* Template implementations.
****************************************************************************/

template<class Type>
usImage3D<Type>::usImage3D() : m_dimx(0), m_dimy(0), m_dimz(0), m_size(0),
m_data(NULL), m_elementSpacingX(1.0f), m_elementSpacingY(1.0f), m_elementSpacingZ(1.0f)
{

}

template<class Type>
usImage3D<Type>::usImage3D(unsigned int dimx, unsigned int dimy, unsigned int dimz,
  float m_elementSpacingX, float m_elementSpacingY, float m_elementSpacingZ)
{
  m_dimx = dimx;
  m_dimy = dimy;
  m_dimz = dimz;
  m_size = m_dimx * m_dimy * m_dimz;
  m_elementSpacingX = m_elementSpacingX;
  m_elementSpacingY = m_elementSpacingY;
  m_elementSpacingZ = m_elementSpacingZ;
  //try to fill
  try {
    m_data = new Type[m_size];
    std::fill_n(m_data, m_size, Type());
  }
  catch(std::bad_alloc) {
    std::cout << "Bad allocation for volume data storage." << std::endl;
  }
}

template<class Type>
usImage3D<Type>::usImage3D(const usImage3D<Type> &volume, const bool copy)
{
  m_dimx = volume.m_dimx;
  m_dimy = volume.m_dimy;
  m_dimz = volume.m_dimz;
  m_size = m_dimx * m_dimy * m_dimz;
  m_elementSpacingX = volume.m_elementSpacingX;
  m_elementSpacingY = volume.m_elementSpacingY;
  m_elementSpacingZ = volume.m_elementSpacingZ;
  //try to allocate memory for data
  try {
    m_data = new Type[m_size];
  }
  catch(std::bad_alloc) {

  }
  if (copy) {
    // not using memcpy because it's a C function, so no exeption thrown.
    //memcpy(m_data, volume.m_data, m_size * sizeof(Type));


  }
}

template<class Type>
usImage3D<Type>::~usImage3D()
{
  if (m_data)
  {
    delete[] m_data;
    m_data = NULL;
  }
}

template<class Type>
usImage3D<Type> &usImage3D<Type>::operator=(const usImage3D<Type> &other)
{
  if (m_dimx != other.m_dimx
    || m_dimy != other.m_dimy
    || m_dimz != other.m_dimz)
  {
    m_dimx = other.m_dimx;
    m_dimy = other.m_dimy;
    m_dimz = other.m_dimz;
    m_size = m_dimx * m_dimy * m_dimz;
    if (m_data) delete[]m_data;
    m_data = new Type[m_size];
  }

  m_elementSpacingX = other.m_elementSpacingX;
  m_elementSpacingY = other.m_elementSpacingY;
  m_elementSpacingZ = other.m_elementSpacingZ;
  memcpy(m_data, other.m_data, m_size * sizeof(Type));
  return *this;
}

template<class Type>
void usImage3D<Type>::setData(Type* data)
{
  m_data = data;
}

template<class Type>
void usImage3D<Type>::initData(Type value)
{
  try {
    std::fill_n(m_data, m_size, value);
  }
  catch (std::bad_alloc)
  {
    std::cout << "Bad allocation using std::fill_n() method." << std::endl;
  }
}
#endif //US_IMAGE_3D_H
