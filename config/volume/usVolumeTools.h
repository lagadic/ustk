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
 * @file usVolumeTools.h
 * @brief Volume handling.
 * @author Pierre Chatelain
 */

#ifndef US_VOLUME_TOOLS_H
#define US_VOLUME_TOOLS_H

#include <float.h>
#include <math.h>
#include <cstring>

#include <visp/vpImage.h>

#include "usVolume.h"

/**
 * @namespace usVolumeTools
 * @brief Basic tools for the usVolume class.
 *
 * This namespace contains basic functions to manipulate the usVolume class.
 */
namespace usVolumeTools
{
  /**
   * Cast the volume to float.
   */
  template<class DataType>
    usVolume<float> castToFloat(usVolume<DataType>& vol);

  /**
   * Cast the volume to float.
   */
  template<class DataType>
    void castFromFloat(usVolume<float>* src, usVolume<DataType>* dst);

  /**
   * Cast the volume to double.
   */
  template<class DataType>
    void castFromDouble(usVolume<double>* src, usVolume<DataType>* dst);

  /**
   * Extract the minimal volume containing all non-zero data.
   */
  template<class DataType>
    usVolume<DataType> extractMinimalROI(usVolume<DataType>& vol);

  /**
   * Find the minimal bounding volume containing all non-zero data.
   */
  template<class DataType>
    void computeMinimalROI(usVolume<DataType>& vol, unsigned int roi[6]);

  /**
   * Insert a volume into another.
   */
  template<class DataType>
    void insertVolume(usVolume<DataType>& vol, usVolume<DataType>& insert);

  /**
   * Check whether two volume have the same extent.
   */
  template<class DataType1, class DataType2>
    bool haveSameExtent(usVolume<DataType1>& vol1, usVolume<DataType2>& vol2);

  /**
   * Check whether the volumes of the list have the same extent.
   */
  template<class DataType>
    bool haveSameExtent(usVolume<DataType>* volumes, unsigned int nVolumes);

  /**
   * Append a new volume to a volume of vectors.
   */
  template<class DataType>
    void append(usVolume<std::vector<DataType> >* container, usVolume<DataType>* volume);

  /**
   * Fill the holes (?)
   */
  template<class DataType>
    void fillHoles(usVolume<DataType>* vol);

  /**
   * Extract a slice from the volume.
   */
  template<class DataType>
    void extractSlice(const usVolume<DataType> &Src, vpImage<DataType> &Dst, unsigned int z,
		      unsigned int downsample = 1);

  /**
   * Insert a slice into a volume.
   */
  template<class DataType>
    void insertSlice(const vpImage<DataType> &Src, usVolume<DataType> &Dst, unsigned int z);

  void findMidbrainROI(usVolume<unsigned char>* labels, const unsigned int& z1,
		       const unsigned int&z2, unsigned int* roi);

  /**
   * Compute the dice score.
   */
  void computeDice(usVolume<unsigned char>* pred, usVolume<unsigned char>* gt, double* dice,
		   const unsigned char &nbClasses);

  /**
   * Downsample the volume.
   */
  template<class DataType>
    void downsample(const usVolume<DataType> &Src, usVolume<DataType> &Dst,
		    unsigned int dx, unsigned int dy, unsigned int dz);

  /**************************************************************
   * Implementations
   **************************************************************/
  
  template<class DataType>
    usVolume<float> castToFloat(usVolume<DataType>& vol)
    {
      unsigned int dimx(vol->getDimX()), dimy(vol->getDimY()), dimz(vol->getDimZ());
      float esx(vol->getElementSpacingX()), esy(vol->getElementSpacingY()), esz(vol->getElementSpacingZ());
      usVolume<float> out = usVolume<float>(dimx, dimy, dimz, esx, esy, esz);
      for (unsigned int i=0; i<vol->getSize(); i++)
	out(i) = static_cast<float>(vol(i));
      return out;
    }
  
  template<class DataType>
    void castFromFloat(usVolume<float>* src, usVolume<DataType>* dst)
    {
      for (unsigned int i=0; i<src->getSize(); i++)
	dst->getData()[i] = static_cast<DataType>(src->getData()[i]);
    }
  
  template<class DataType>
    void castFromDouble(usVolume<double>* src, usVolume<DataType>* dst)
    {
      for (unsigned int i=0; i<src->getSize(); i++)
	dst->getData()[i] = static_cast<DataType>(src->getData()[i]);
    }
  
  template<class DataType>
    usVolume<DataType> extractMinimalROI(usVolume<DataType>& vol)
    {
      unsigned int dimx(vol.getDimX()), dimy(vol.getDimY()), dimz(vol.getDimZ());
      float esx(vol.getElementSpacingX()), esy(vol.getElementSpacingY()), esz(vol.getElementSpacingZ());
      unsigned int roi[6];
      roi[0] = 0;
      roi[1] = dimx-1;
      roi[2] = 0;
      roi[3] = dimy-1;
      roi[4] = 0;
      roi[5] = dimz-1;
      DataType zero(0);
      bool empty = true;
      unsigned int x, y, z;
      y = 0;
      z = 0;
      while(empty && roi[0]<roi[1]) // X1
	{
	  if (vol(roi[0],y,z)>zero) empty = false;
	  else if (y == dimy-1 && z == dimz-1) { roi[0]++; y=0; z=0; }
	  else if (z == dimz-1) { y++; z=0; }
	  else z++;
	}
      empty = true;
      y = 0;
      z = 0;
      while(empty && roi[0]<roi[1]) // X2
	{
	  if (vol(roi[1],y,z)>zero) empty = false;
	  else if (y == dimy-1 && z == dimz-1) { roi[1]--; y=0; z=0; }
	  else if (z == dimz-1) { y++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      z = 0;
      while(empty && roi[2]<roi[3]) // Y1
	{
	  if (vol(x,roi[2],z)>zero) empty = false;
	  else if (x == dimx-1 && z == dimz-1) { roi[2]++; x=0; z=0; }
	  else if (z == dimz-1) { x++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      z = 0;
      while(empty && roi[2]<roi[3]) // Y2
	{
	  if (vol(x,roi[3],z)>zero) empty = false;
	  else if (x == dimx-1 && z == dimz-1) { roi[3]--; x=0; z=0; }
	  else if (z == dimz-1) { x++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      y = 0;
      while(empty && roi[4]<roi[5]) // Z1
	{
	  if (vol(x,y,roi[4])>zero) empty = false;
	  else if (x == dimx-1 && y == dimy-1) { roi[4]++; x=0; y=0; }
	  else if (y == dimy-1) { x++; y=0; }
	  else y++;
	}
      empty = true;
      x = 0;
      y = 0;
      while(empty && roi[4]<roi[5]) // Z2
	{
	  if (vol(x,y,roi[5])>zero) empty = false;
	  else if (x == dimx-1 && y == dimy-1) { roi[5]--; x=0; y=0; }
	  else if (y == dimy-1) { x++; y=0; }
	  else y++;
	}
      if (roi[0]==roi[1] || roi[2]==roi[3] || roi[4]==roi[5])
	std::cerr << "Error: empty ROI." << std::endl;
      dimx = roi[1]-roi[0]+1;
      dimy = roi[3]-roi[2]+1;
      dimz = roi[5]-roi[4]+1;
      usVolume<DataType> out(dimx, dimy, dimz, esx, esy, esz);
      for (unsigned int x = roi[0]; x<=roi[1]; x++)
	for (unsigned int y = roi[2]; y<=roi[3]; y++)
	  for (unsigned int z = roi[4]; z<=roi[5]; z++)
	    out(x-roi[0],y-roi[2],z-roi[4]) = vol(x,y,z);
      out.setOriginX(esx*roi[0]);
      out.setOriginY(esy*roi[2]);
      out.setOriginZ(esz*roi[4]);
      return out;
    }
  /*
  template<>
    void computeMinimalROI(usVolume<unsigned char>& vol, unsigned int roi[6])
    {
      unsigned int dimx(vol.getDimX()), dimy(vol.getDimY()), dimz(vol.getDimZ());
      roi[0] = 0;
      roi[1] = dimx-1;
      roi[2] = 0;
      roi[3] = dimy-1;
      roi[4] = 0;
      roi[5] = dimz-1;
      bool empty = true;
      unsigned int x, y, z;
      y = 0;
      z = 0;
      while(empty && roi[0]<roi[1]) // X1
	{
	  if (vol(roi[0],y,z)) empty = false;
	  else if (y == dimy-1 && z == dimz-1) { roi[0]++; y=0; z=0; }
	  else if (z == dimz-1) { y++; z=0; }
	  else z++;
	}
      empty = true;
      y = 0;
      z = 0;
      while(empty && roi[0]<roi[1]) // X2
	{
	  if (vol(roi[1],y,z)) empty = false;
	  else if (y == dimy-1 && z == dimz-1) { roi[1]--; y=0; z=0; }
	  else if (z == dimz-1) { y++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      z = 0;
      while(empty && roi[2]<roi[3]) // Y1
	{
	  if (vol(x,roi[2],z)) empty = false;
	  else if (x == dimx-1 && z == dimz-1) { roi[2]++; x=0; z=0; }
	  else if (z == dimz-1) { x++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      z = 0;
      while(empty && roi[2]<roi[3]) // Y2
	{
	  if (vol(x,roi[3],z)) empty = false;
	  else if (x == dimx-1 && z == dimz-1) { roi[3]--; x=0; z=0; }
	  else if (z == dimz-1) { x++; z=0; }
	  else z++;
	}
      empty = true;
      x = 0;
      y = 0;
      while(empty && roi[4]<roi[5]) // Z1
	{
	  if (vol(x,y,roi[4])) empty = false;
	  else if (x == dimx-1 && y == dimy-1) { roi[4]++; x=0; y=0; }
	  else if (y == dimy-1) { x++; y=0; }
	  else y++;
	}
      empty = true;
      x = 0;
      y = 0;
      while(empty && roi[4]<roi[5]) // Z2
	{
	  if (vol(x,y,roi[5])) empty = false;
	  else if (x == dimx-1 && y == dimy-1) { roi[5]--; x=0; y=0; }
	  else if (y == dimy-1) { x++; y=0; }
	  else y++;
	}
      if (roi[0]==roi[1] || roi[2]==roi[3] || roi[4]==roi[5])
	std::cerr << "Error: empty ROI." << std::endl;
    }
  */
  template<class DataType>
    void insertVolume(usVolume<DataType>& vol, usVolume<DataType>& insert)
    {
      // Big volume
      unsigned int dimx(vol.getDimX()), dimy(vol.getDimY()), dimz(vol.getDimZ());
      float esx(vol.getElementSpacingX()), esy(vol.getElementSpacingY()), esz(vol.getElementSpacingZ());
      // Small volume
      unsigned int dx(insert.getDimX()), dy(insert.getDimY()), dz(insert.getDimZ());
      float ex(insert.getElementSpacingX()), ey(insert.getElementSpacingY()), ez(insert.getElementSpacingZ());
      unsigned int x0(floor(insert.getOriginX()/esx)), y0(floor(insert.getOriginY()/esy)), z0(floor(insert.getOriginZ()/esz));
      // Check element spacing
      if (esx!=ex || esy!=ey || esz!=ez)
	{
	  std::cerr << "ERROR: element spacing mismatch." << std::endl;
	  return;
	}
      // Check insertion area
      if (x0+dx>dimx || y0+dy>dimy || z0+dz>dimz)
	{
	  std::cerr << "ERROR: inserted volume doesn't fit in the volume extent." << std::endl;
	  return;
	}
      // Insert volume
      for (unsigned int x=x0; x<x0+dx; x++)
	for (unsigned int y=y0; y<y0+dy; y++)
	  for (unsigned int z=z0; z<z0+dz; z++)
	    vol(x,y,z) = insert(x-x0,y-y0,z-z0);
    }

  template<class DataType1, class DataType2>
    bool haveSameExtent(usVolume<DataType1>& vol1, usVolume<DataType2>& vol2)
  {
    unsigned int dimx1(vol1.getDimX()), dimy1(vol1.getDimY()), dimz1(vol1.getDimZ());
    float esx1(vol1.getElementSpacingX()), esy1(vol1.getElementSpacingY()), esz1(vol1.getElementSpacingZ());
    float ox1(vol1.getOriginX()), oy1(vol1.getOriginY()), oz1(vol1.getOriginZ());
    unsigned int dimx2(vol2.getDimX()), dimy2(vol2.getDimY()), dimz2(vol2.getDimZ());
    float esx2(vol2.getElementSpacingX()), esy2(vol2.getElementSpacingY()), esz2(vol2.getElementSpacingZ());
    float ox2(vol2.getOriginX()), oy2(vol2.getOriginY()), oz2(vol2.getOriginZ());
    return (dimx1==dimx2 && dimy1==dimy2 && dimz1==dimz2 && esx1==esx2 && esy1==esy2 && esz1==esz2 && ox1==ox2 && oy1==oy2 && oz1==oz2);
  }

  template<class DataType>
    bool haveSameExtent(usVolume<DataType>* volumes, unsigned int nVolumes)
    {
      bool out = true;
      for (unsigned int i=1; i<nVolumes; i++)
	out = out && haveSameExtent(volumes[0], volumes[i]);
      return out;
    }

  template<class DataType>
    void append(usVolume<std::vector<DataType> >* container, usVolume<DataType>* volume)
    {
      unsigned int size(volume->getSize());
      for (unsigned int i=0; i<size; i++)
	container->getData()[i].push_back(volume->getData()[i]);
    }

  /*
  template<>
    void fillHoles(usVolume<float>* vol)
    {
      unsigned int dimx(vol->getDimX()), dimy(vol->getDimY()), dimz(vol->getDimZ()), pos, votes;
      for (unsigned int x=1; x<dimx-1; ++x)
	for (unsigned int y=1; y<dimy-1; ++y)
	  for (unsigned int z=1; z<dimz-1; ++z)
	    {
	      pos = x + dimx*y + dimx*dimy*z;
	      if (vol->getData()[pos] < 0.0f)
		{
		  vol->getData()[pos] = 0.0f;
		  votes = 0;
		  if (vol->getData()[pos+1] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos+1];
		      ++votes;
		    }
		  if (vol->getData()[pos-1] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos-1];
		      ++votes;
		    }
		  if (vol->getData()[pos+dimx] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos+dimx];
		      ++votes;
		    }
		  if (vol->getData()[pos-dimx] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos-dimx];
		      ++votes;
		    }
		  if (vol->getData()[pos+dimx*dimy] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos+dimx*dimy];
		      ++votes;
		    }
		  if (vol->getData()[pos-dimx*dimy] >= 0.0f)
		    {
		      vol->getData()[pos] += vol->getData()[pos-dimx*dimy];
		      ++votes;
		    }
		  if (votes>0)
		    vol->getData()[pos] /= votes;
		  else
		    vol->getData()[pos] = 0.0f;
		}
	    }
      unsigned int size = vol->getSize();
      for (unsigned int i=0; i<size; ++i)
	{
	  if (vol->getData()[i] < 0.0f)
	    vol->getData()[i] = 0.0f;
	}
    }
  */
  /*
  void computeDice(usVolume<unsigned char>* pred, usVolume<unsigned char>* gt, double* dice, const unsigned char &nbClasses)
  {
    double** confusion = new double*[nbClasses];
    for (unsigned int i=0; i<nbClasses; ++i)
      {
	confusion[i] = new double[nbClasses];
	for (unsigned int j=0; j<nbClasses; ++j)
	  confusion[i][j] = 0.0;
      }
    for (unsigned j=0; j<pred->getSize(); ++j)
      confusion[gt->getData()[j]][pred->getData()[j]]++;
    double* sumRows = new double[nbClasses];
    double* sumCols = new double[nbClasses];
    for (unsigned int i=0; i<nbClasses; ++i)
      {
	sumRows[i] = 0.0;
	sumCols[i] = 0.0;
      }
    for (unsigned int i=0; i<nbClasses; ++i)
      {
	for (unsigned int j=0; j<nbClasses; ++j)
	  {
	    sumRows[i] += confusion[i][j];
	    sumCols[i] += confusion[j][i];
	  }
      }
    for (unsigned int i=0; i<nbClasses; ++i)
      {
	dice[i] = 2.0 * confusion[i][i] / (sumRows[i] + sumCols[i]);
	delete [] confusion[i];
      }
    delete [] confusion;
    delete [] sumRows;
    delete [] sumCols;
  }
  */
  template<class DataType>
    void extractSlice(const usVolume<DataType> &Src, vpImage<DataType> &Dst, unsigned int z,
		      unsigned int downsample)
    {
      unsigned int height = Src.getDimX() / downsample;
      unsigned int width = Src.getDimY();
      Dst.resize(height, width);
      for (unsigned int x = 0; x < height; ++x)
	for (unsigned int y = 0; y < width; ++y)
	  Dst(x, y, Src(x * downsample, y, z));
      //std::memcpy(Dst.bitmap, Src.getData() + Dst.getSize() * z, Dst.getSize() * sizeof(DataType));
    }

  template<class DataType>
    void insertSlice(const vpImage<DataType> &Src, usVolume<DataType> &Dst, unsigned int z)
    {
      unsigned int height = Src.getHeight();
      unsigned int width = Src.getWidth();
      for (unsigned int x = 0; x < height; ++x)
	for (unsigned int y = 0; y < width; ++y)
	  Dst(x, y, z, Src(x, y));      
    }
  
  template<class DataType>
    void downsample(const usVolume<DataType> &Src, usVolume<DataType> &Dst,
		    unsigned int dx, unsigned int dy, unsigned int dz)
    {
      unsigned int dimx = Src.getDimX() / dx;
      unsigned int dimy = Src.getDimY() / dy;
      unsigned int dimz = Src.getDimZ() / dz;
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int z = 0; z < dimz; ++z)
	for (unsigned int y = 0; y < dimy; ++y)
	  for (unsigned int x = 0; x < dimx; ++x)
	    Dst(x, y, z, Src(x * dx, y * dy, z * dz));
    }
}

#endif // US_VOLUME_TOOLS_H
