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
*
* This class is used to represent 3D data with physical information such as element spacing.
*/

#ifndef US_IMAGE_3D_H
#define US_IMAGE_3D_H

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

/**
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
  * @param dimX size (in pixels) on X-axis
  * @param dimY size (in pixels) on Y-axis
  * @param dimZ size (in pixels) on Z-axis
  */
  usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ);

  /**
  * Constructor. Set the dimensions and element spacing of the volume.
  * @param dimX size (in pixels) on X-axis
  * @param dimY size (in pixels) on Y-axis
  * @param dimZ size (in pixels) on Z-axis
  * @param spacingX distancee (in meters) between two voxels on X-axis
  * @param spacingY distancee (in meters) between two voxels on Y-axis
  * @param spacingZ distancee (in meters) between two voxels on Z-axis
  */
  usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ, float spacingX, float spacingY, float spacingZ);

  /**
  * Copy constructor. By default doesn't perform a deep copy.
  * @param image other 3D-image to copy
  * @param copy boolean to select if deep copy is performed or not (not a deep copy by default)
  */
  usImage3D(const usImage3D<Type> &image, const bool copy = false);

  /**
  * Destructor.
  */
  ~usImage3D();

  /**
  * Assignment operator.
  * @param other other 3D-image to copy
  */
  usImage3D<Type> &operator=(const usImage3D<Type> &other);

  /**
  * Comparaison operator.
  * @param other The 3d image to compare. Comparing only parameters, not all volume voxel by voxel.
  */
  bool operator==(const usImage3D<Type> &other);

  /**
  * Access operator.
  * @param index Index of the data to acess.
  */
  inline Type operator()(unsigned int index) const
  {
    return bitmap[index];
  }

  //! operator[] allows operation like I[i] = x.
  Type& operator[]( const unsigned int i)   { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}
  Type& operator[]( const int i) { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}

  //! operator[] allows operation like x = I[i]
  const  Type& operator[](unsigned int i) const { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}
  const  Type& operator[](int i) const { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}


  /**
  * initiation of the image.
  * @param h volume height.
  * @param w volume width.
  * @param d volume depth.
  */
  void init(unsigned int h, unsigned int w, unsigned int d);

  /**
  * initiation of the image.
  * @param h volume height.
  * @param w volume width.
  * @param d volume depth.
  * @param spacingX Element spacing in x axis.
  * @param spacingY Element spacing in x axis.
  * @param spacingZ Element spacing in x axis.
  */
  void init(unsigned int h, unsigned int w, unsigned int d, float spacingX, float spacingY, float spacingZ);

  /**
  * Modification operator.
  * @param index Index of the data to modify.
  * @param value New value to set.
  */
  inline void operator()(unsigned int index, Type value)
  {
    bitmap[index] = value;
  }

  /**
  * Access operator.
  * @param indexX Index on x-axis to acess
  * @param indexY Index on y-axis to acess
  * @param indexZ Index on z-axis to acess
  */
  inline Type operator()(unsigned int indexX, unsigned int indexY, unsigned int indexZ) const
  {
    return bitmap[(m_dimx * m_dimy) * indexZ + m_dimx*indexY + indexX];
  }

  /**
  * Modification operator.
  * @param indexX Index on x-axis to modify
  * @param indexY Index on y-axis to modify
  * @param indexZ Index on z-axis to modify
  * @param value Value to insert at the desired index
  */
  inline void operator()(unsigned int indexX, unsigned int indexY, unsigned int indexZ, Type value)
  {
    bitmap[(m_dimx * m_dimy)*indexZ + m_dimx*indexY + indexX] = value;
  }

  /**
  * Get the volume size.
  * @return The number of voxels in the volume.
  */
  unsigned int getSize() const { return m_size; }

  /**
  * Get the volume height.
  * @return The number of pixel on the x-axis.
  */
  unsigned int getDimX() const { return m_dimx; }

  /**
  * Get the volume width.
  * @return The number of pixel on the y-axis.
  */
  unsigned int getDimY() const { return m_dimy; }

  /**
  * Get the volume depth.
  * @return The number of pixel on the z-axis.
  */
  unsigned int getDimZ() const { return m_dimz; }

  /**
  * Get the element spacing along the x-axis.
  * @return The element spacing along the x-axis
  */
  float getElementSpacingX() const { return m_elementSpacingX; }

  /**
  * Get the element spacing along the y-axis.
  * @return The element spacing along the y-axis
  */
  float getElementSpacingY() const { return m_elementSpacingY; }

  /**
  * Get the element spacing along the z-axis.
  * @return The element spacing along the z-axis
  */
  float getElementSpacingZ() const { return m_elementSpacingZ; }

  /**
  * Set the element spacing along the x-axis.
  * @param elementSpacingX The element spacing along the x-axis
  */
  void setElementSpacingX(float elementSpacingX) { m_elementSpacingX = elementSpacingX; }

  /**
  * Set the element spacing along the y-axis.
  * @param elementSpacingY The element spacing along the y-axis
  */
  void setElementSpacingY(float elementSpacingY) { m_elementSpacingY = elementSpacingY; }

  /**
  * Set the element spacing along the z-axis.
  * @param elementSpacingZ The element spacing along the z-axis
  */
  void setElementSpacingZ(float elementSpacingZ) { m_elementSpacingZ = elementSpacingZ; }

  /**
  * Get the pointer to the data container.
  * @return The pointer to the data container
  */
  Type* getData() { return bitmap; }

  /**
  * Get the pointer to the const data container.
  * @return The pointer to the const data container
  */
  Type* getConstData() const { return bitmap; }

  /**
  * Set the data container.
  * @param data The data container
  */
  void setData(Type* data, int numberOfVloxels);

  /**
  * Initialize the data container with the specified value.
  * @param value The data
  * @param numberOfVloxels number of voxels in the volume
  */
  void initData(Type value,int numberOfVloxels);


  void resize(unsigned int dimx,unsigned int dimy,unsigned int dimz);

protected:

private:
  unsigned int m_dimx; /**< Volume height in pixels (number of pixels on the x-axis)*/
  unsigned int m_dimy; /**< Volume width in pixels (number of pixels on the x-axis)*/
  unsigned int m_dimz; /**< Volume depth in pixels (number of pixels on the x-axis)*/
  float m_elementSpacingX; /**< Element spacing along the x-axis, in meters */
  float m_elementSpacingY; /**< Element spacing along the y-axis, in meters */
  float m_elementSpacingZ; /**< Element spacing along the z-axis, in meters */
  unsigned int m_size; /**< Volume size : number of voxels in the whole volume*/

  Type* bitmap; /**< Data container */
  Type **planesIndex ;
};

/****************************************************************************
* Template implementations.
****************************************************************************/
/*!
  \brief Image initialization

  Allocate memory for an [h x w x d] image.

  \param w : Image width.
  \param h : Image height.
  \param d : Image depth.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template<class Type>
void
usImage3D<Type>::init(unsigned int h, unsigned int w, unsigned int d)
{
  if ((h != this->m_dimx) || (w != this->m_dimy)) {
    if (planesIndex != NULL)  {
      vpDEBUG_TRACE(10,"Destruction index[]");
      delete [] planesIndex;
      planesIndex = NULL;
    }
  }

  if ((h != this->m_dimx) || (w != this->m_dimy) || (d != this->m_dimz))
  {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10,"Destruction bitmap[]") ;
      delete [] bitmap;
      bitmap = NULL;
    }
  }

  this->m_dimx = h;
  this->m_dimy = w;
  this->m_dimz = d;

  m_size=m_dimx*m_dimy*m_dimz;

  if (bitmap == NULL)  bitmap = new  Type[m_size] ;

  //  vpERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL)
  {
    vpERROR_TRACE("cannot allocate bitmap ") ;
    throw(vpException(vpException::memoryAllocationError,
                      "cannot allocate bitmap ")) ;
  }

  if (planesIndex == NULL)  planesIndex = new  Type*[m_dimx];
  //  vpERROR_TRACE("Allocate row %p",row) ;
  if (planesIndex == NULL)
  {
    vpERROR_TRACE("cannot allocate index ") ;
    throw(vpException(vpException::memoryAllocationError,
                      "cannot allocate index ")) ;
  }

  //filling planesIndex
  unsigned int i ;
  for ( i =0  ; i < m_dimz ; i++)
    planesIndex[i] = bitmap + i*m_dimx*m_dimy ;
}

template<class Type>
void
usImage3D<Type>::init(unsigned int h, unsigned int w, unsigned int d, float spacingX, float spacingY, float spacingZ)
{
  init(h,w,d);
  m_elementSpacingX = spacingX;
  m_elementSpacingY = spacingY;
  m_elementSpacingZ = spacingZ;
}

template<class Type>
usImage3D<Type>::usImage3D() : m_dimx(0), m_dimy(0), m_dimz(0), m_elementSpacingX(1.0f), m_elementSpacingY(1.0f), m_elementSpacingZ(1.0f),
  m_size(0),bitmap(NULL), planesIndex(NULL)
{

}

template<class Type>
usImage3D<Type>::usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ) : m_dimx(dimX), m_dimy(dimY), m_dimz(dimZ),
  m_size(dimX * dimY * dimZ),m_elementSpacingX(1.0f), m_elementSpacingY(1.0f), m_elementSpacingZ(1.0f), bitmap(NULL), planesIndex(NULL) {
  init(dimX,dimY,dimZ);
}

template<class Type>
usImage3D<Type>::usImage3D(unsigned int dimx, unsigned int dimy, unsigned int dimz,
                           float elementSpacingX, float elementSpacingY, float elementSpacingZ)  : m_dimx(dimx), m_dimy(dimy), m_dimz(dimz), m_size(dimx * dimy * dimz),
  m_elementSpacingX(elementSpacingX), m_elementSpacingY(elementSpacingY), m_elementSpacingZ(elementSpacingZ), bitmap(NULL), planesIndex(NULL) {
  init(dimx,dimy,dimz);
}

template<class Type>
usImage3D<Type>::usImage3D(const usImage3D<Type> &volume, const bool copy)
{
  init(volume.getDimX(),volume.getDimY(),volume.getDimZ());

  m_size = m_dimx * m_dimy * m_dimz;
  m_elementSpacingX = volume.getElementSpacingX();
  m_elementSpacingY = volume.getElementSpacingY();
  m_elementSpacingZ = volume.getElementSpacingZ();

  //deep copy
  if(copy)
    memcpy(bitmap, volume.bitmap, m_size * sizeof(Type));
}

template<class Type>
usImage3D<Type>::~usImage3D()
{
  if (planesIndex)
  {
    delete[] planesIndex;
    planesIndex = NULL;
  }
  if (bitmap)
  {
    delete[] bitmap;
    bitmap = NULL;
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
    if (bitmap) delete[]bitmap;
    bitmap = new Type[m_size];
  }

  m_elementSpacingX = other.m_elementSpacingX;
  m_elementSpacingY = other.m_elementSpacingY;
  m_elementSpacingZ = other.m_elementSpacingZ;
  memcpy(bitmap, other.bitmap, m_size * sizeof(Type));
  return *this;
}

template<class Type>
bool usImage3D<Type>::operator==(const usImage3D<Type> &other)
{
  bool settingsOk = this->getDimX() == other.getDimX() &&
      this->getDimY() == other.getDimY() &&
      this->getDimZ() == other.getDimZ() &&
      this->getElementSpacingX() == other.getElementSpacingX() &&
      this->getElementSpacingY() == other.getElementSpacingY() &&
      this->getElementSpacingZ() == other.getElementSpacingZ();

  if(settingsOk) {
    for (unsigned int i=0 ; i < m_size ; i++) {
      if (bitmap[i] != other[i])
        return false;
    }
  }
  return true;
}

template<class Type>
void usImage3D<Type>::setData(Type* data, int numberOfVloxels)
{
  try {
    m_size =numberOfVloxels;
    memcpy(bitmap, data, m_size * sizeof(Type));
  }
  catch (std::exception e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "Bad allocation using std::fill_n() method." << std::endl;
  }
}

template<class Type>
void usImage3D<Type>::initData(Type value,int numberOfVloxels)
{
  try {
    m_size =numberOfVloxels;
    std::fill_n(bitmap, m_size, value);
  }
  catch (std::exception e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "Bad allocation using std::fill_n() method." << std::endl;
  }
}

template<class Type>
void usImage3D<Type>::resize(unsigned int dimx,unsigned int dimy,unsigned int dimz)
{
  init(dimx,dimy,dimz);
}

#endif //US_IMAGE_3D_H
