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
 *
 *****************************************************************************/

/**
 * @file usVolume.h
 * @brief Volume handling.
 * @author Pierre Chatelain
 */

#ifndef US_VOLUME_H
#define US_VOLUME_H

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

/**
 * Storage element type for MetaImage (mhd) format.
 */
enum ElementType
  {
    MET_UNKNOWN, MET_CHAR, MET_UCHAR, MET_SHORT, MET_USHORT, MET_LONG, MET_ULONG, MET_INT, MET_UINT,
    MET_FLOAT, MET_DOUBLE, MET_VECTOR
  };

/**
 * @class usVolume
 * @brief Representation of a physical data volume.
 *
 * This class is used to represent 3D data with physical information such as element spacing.
 */
template <class DataType>
class VISP_EXPORT usVolume
{
 public:
  /**
   * Constructor.
   */
  usVolume();

  /**
   * Constructor. Set the dimensions and element spacing of the volume.
   */
  usVolume(unsigned int dimx, unsigned int dimy, unsigned int dimz, float esx, float esy, float esz);

  /**
   * Copy constructor. By default performs a deep copy.
   */
  usVolume(const usVolume<DataType> &volume, const bool copy=true);

  /**
   * Destructor.
   */
  ~usVolume();

  /**
   * Assignment operator.
   */
  usVolume<DataType> &operator=(const usVolume<DataType> &other);

  /**
   * Access operator.
   */
  inline DataType operator()(unsigned int ind) const
  {
    return m_data[ind];
  }

  /**
   * Modification operator.
   */
  inline void operator()(unsigned int ind, DataType v)
  {
    m_data[ind] = v;
  }

  /**
   * Access operator.
   */
  inline DataType operator()(unsigned int x, unsigned int y, unsigned int z) const
  {
    return m_data[m_dimxy*z + m_dimx*y + x];
  }

  /**
   * Modification operator.
   */
  inline void operator()(unsigned int x, unsigned int y, unsigned int z, DataType v)
  {
    m_data[m_dimxy*z + m_dimx*y + x] = v;
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
  float getElementSpacingX() const { return m_esx; }

  /**
   * Get the element spacing along the y-axis.
   */
  float getElementSpacingY() const { return m_esy; }

  /**
   * Get the element spacing along the y-axis.
   */
  float getElementSpacingZ() const { return m_esz; }

  /**
   * Set the element spacing along the x-axis.
   */
  void setElementSpacingX(float s) { m_esx = s; }

  /**
   * Set the element spacing along the y-axis.
   */
  void setElementSpacingY(float s) { m_esy = s; }

  /**
   * Set the element spacing along the y-axis.
   */
  void setElementSpacingZ(float s) { m_esz = s; }

  /**
   * Get the storage element type.
   */
  ElementType getElementType() const { return m_elementType; }

  /**
   * Set the storage element type.
   */
  void setElementType();

  /**
   * Get the pointer to the data container.
   */
  DataType* getData() { return m_data; }

  /**
   * Get the pointer to the const data container.
   */
  const DataType* getConstData() const { return m_data; }

  /**
     * Set the data container.
     */
    void setData(DataType* data);

  /** Resize the volume */
  void resize(unsigned int dimx, unsigned int dimy, unsigned int dimz);

  /** Resize the volume */
  void resize(unsigned int dims[3]);

  /**
   * Initialize the data container with the specified value.
   */
  void initData(DataType value = DataType());

 protected:
  unsigned int m_dimx; /**< Volume height. */
  unsigned int m_dimy; /**< Volume width. */
  unsigned int m_dimz; /**< Volume depth. */
  unsigned int m_dimxy; /**< Size of a volume slice. */
  unsigned int m_size; /**< Volume size. */
  DataType* m_data; /**< Data container. */
  float m_esx; /**< Element spacing along the x-axis. */
  float m_esy; /**< Element spacing along the y-axis. */
  float m_esz; /**< Element spacing along the z-axis. */
  ElementType m_elementType; /**< Storage element type. */
};

/****************************************************************************
 * Template implementations.
 ****************************************************************************/

template<class DataType>
usVolume<DataType>::usVolume() : m_dimx(0), m_dimy(0), m_dimz(0), m_dimxy(0), m_size(0),
  m_data(NULL), m_esx(1.0f), m_esy(1.0f), m_esz(1.0f)
{
  setElementType();
}

template<class DataType>
usVolume<DataType>::usVolume(unsigned int dimx, unsigned int dimy, unsigned int dimz,
			     float esx, float esy, float esz)
{
  m_dimx = dimx;
  m_dimy = dimy;
  m_dimz = dimz;
  m_dimxy = m_dimx * m_dimy;
  m_size = m_dimxy * m_dimz;
  m_esx = esx;
  m_esy = esy;
  m_esz = esz;
  setElementType();
  m_data = new DataType[m_size];
  std::fill_n(m_data, m_size, DataType());
}

template<class DataType>
usVolume<DataType>::usVolume(const usVolume<DataType> &volume, const bool copy)
{
  m_dimx = volume.m_dimx;
  m_dimy = volume.m_dimy;
  m_dimz = volume.m_dimz;
  m_dimxy = m_dimx * m_dimy;
  m_size = m_dimxy * m_dimz;
  m_esx = volume.m_esx;
  m_esy = volume.m_esy;
  m_esz = volume.m_esz;
  m_elementType = volume.m_elementType;
  m_data = new DataType[m_size];
  if(copy) memcpy(m_data, volume.m_data, m_size*sizeof(DataType));
}

template<class DataType>
usVolume<DataType>::~usVolume()
{
  if(m_data)
  {
    delete[] m_data;
    m_data = NULL;
  }
}
#include <visp3/core/vpTime.h>
template<class DataType>
usVolume<DataType> &usVolume<DataType>::operator=(const usVolume<DataType> &other)
{
  if(   m_dimx != other.m_dimx
     || m_dimy != other.m_dimy
     || m_dimz != other.m_dimz)
  {
      m_dimx = other.m_dimx;
      m_dimy = other.m_dimy;
      m_dimz = other.m_dimz;
      m_dimxy = m_dimx * m_dimy;
      m_size = m_dimxy * m_dimz;
      if(m_data) delete []m_data;
      m_data = new DataType[m_size];
  }

  m_esx = other.m_esx;
  m_esy = other.m_esy;
  m_esz = other.m_esz;
  m_elementType = other.m_elementType;
  memcpy(m_data, other.m_data, m_size*sizeof(DataType));
  return *this;
}

template<class DataType>
void usVolume<DataType>::setData(DataType* data)
{
  m_data = data;
}

template<class DataType>
void usVolume<DataType>::resize(unsigned int dimx, unsigned int dimy, unsigned int dimz)
{
  if(   m_dimx != dimx
     || m_dimy != dimy
     || m_dimz != dimz)
  {
    m_dimx = dimx;
    m_dimy = dimy;
    m_dimz = dimz;
    m_dimxy = m_dimx * m_dimy;
    m_size = m_dimxy * m_dimz;
    if (m_data) delete [] m_data;
    m_data = new DataType[m_size];
  }
}

template<class DataType>
void usVolume<DataType>::resize(unsigned int dims[3])
{
    this->resize(dims[0], dims[1], dims[2]);
}

template<class DataType>
void usVolume<DataType>::initData(DataType value)
{
  std::fill_n(m_data, m_size, value);
}

template<class T> inline void usVolume<T>::setElementType() {
  m_elementType = MET_UNKNOWN;
}

template<> inline void usVolume<char>::setElementType() {
  m_elementType = MET_CHAR;
}

template<> inline void usVolume<unsigned char>::setElementType() {
  m_elementType = MET_UCHAR;
}

template<> inline void usVolume<short>::setElementType() {
  m_elementType = MET_SHORT;
}

template<> inline void usVolume<unsigned short>::setElementType() {
  m_elementType = MET_USHORT;
}

template<> inline void usVolume<int>::setElementType() {
  m_elementType = MET_INT;
}

template<> inline void usVolume<unsigned int>::setElementType() {
  m_elementType = MET_UINT;
}

template<> inline void usVolume<long>::setElementType() {
  m_elementType = MET_LONG;
}

template<> inline void usVolume<unsigned long>::setElementType() {
  m_elementType = MET_ULONG;
}

template<> inline void usVolume<float>::setElementType() {
  m_elementType = MET_FLOAT;
}

template<> inline void usVolume<double>::setElementType() {
  m_elementType = MET_DOUBLE;	
}

template<> inline void usVolume<std::vector<unsigned char> >::setElementType() {
  m_elementType = MET_VECTOR;
}

#endif
