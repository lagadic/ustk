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
 * @file usVolumeProcessing.h
 * @brief Volume processing.
 * @author Pierre Chatelain
 */

#ifndef US_VOLUME_PROCESSING_H
#define US_VOLUME_PROCESSING_H

#include <float.h>
#include <cmath>
#include <cstring>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include "usVolume.h"

/**
 * @namespace usVolumeProcessing
 * @brief Processing tools for the usVolume class.
 *
 * This namespace contains processing tools (derivative, filtering...) for the usVolume class.
 */
namespace usVolumeProcessing
{
  /**
   * Get the max value.
   */
  template<class T>
    T max(const usVolume<T> &V);

  /**
   * Apply a filter to a voxel.
   */
  template<class T>
    double applyFilter(const usVolume<T> &Src, const usVolume<double> F,
		       unsigned int x, unsigned int y, unsigned int z);

  /**
   * Apply a filter to a volume.
   */
  template<class T>
    void applyFilter(const usVolume<T> &Src, usVolume<double> &Dst,
		     const usVolume<double> F);

  /**
   * Apply a 3x1x1 derivative filter to a voxel.
   */
  template<class T>
    double derivativeX(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Apply a 1x3x1 derivative filter to a voxel.
   */
  template<class T>
    double derivativeY(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Apply a 1x1x3 derivative filter to a voxel.
   */
  template<class T>
    double derivativeZ(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Compute the x-derivative of a volume
   */
  template<class T>
    void derivativeX_2(const usVolume<T> &Src, usVolume<double> &Dst,
		       unsigned int filter_size, double sigma);

  /**
   * Apply a 3x1x1 derivative filter to a volume.
   */
  template<class T>
    void derivativeX(const usVolume<T> &Src, usVolume<double> &Dst);

  /**
   * Compute the volume gradient.
   */
  template<class T>
    void gradient(const usVolume<T> &Src, usVolume<vpColVector> &Dst);

  /**
   * Compute the volume hessian.
   */
  template<class T>
    void hessian(const usVolume<T> &Src, usVolume<vpMatrix> &Dst);

  /**
   * Compute the Frangi's vesselness.
   */
  template<class T>
    void frangi(const usVolume<T> &Src, usVolume<double> &Dst,
		double a, double b, double c);

  /**
   * Compute the norm of a vector image.
   */
  void norm(const usVolume<vpColVector> &Src, usVolume<double> &Dst);

  /**
   * Compute the volume difference.
   */
  template<class T1, class T2>
    void difference(const usVolume<T1> &Src1, const usVolume<T1> &Src2, usVolume<T2> &Dst);

  /**
   * Compute the volume absolute difference.
   */
  template<class T1, class T2>
    void absoluteDifference(const usVolume<T1> &Src1, const usVolume<T1> &Src2, usVolume<T2> &Dst);

  /**
   * Generate a Gaussian derivative filter in the x-direction.
   */
  usVolume<double> getDerivativeFilterX(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the y-direction.
   */
  usVolume<double> getDerivativeFilterY(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the z-direction.
   */
  usVolume<double> getDerivativeFilterZ(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the x-direction.
   */
  usVolume<double> getDerivativeFilterXX(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the y-direction.
   */
  usVolume<double> getDerivativeFilterYY(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the z-direction.
   */
  usVolume<double> getDerivativeFilterZZ(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the x-direction.
   */
  usVolume<double> getDerivativeFilterXY(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the y-direction.
   */
  usVolume<double> getDerivativeFilterXZ(double sigma, unsigned int size);

  /**
   * Generate a Gaussian derivative filter in the z-direction.
   */
  usVolume<double> getDerivativeFilterYZ(double sigma, unsigned int size);

  /**
   * Compute the photometric barycenter of the volume.
   */
  void computeBarycenter(const usVolume<unsigned char> &V, double &xc, double &yc, double &zc);

  /**************************************************************
   * Instanciations
   **************************************************************/

  template<class T>
    T max(const usVolume<T> &V)
    {
      T max = V(0);
      for (unsigned int i = 1; i < V.getSize(); ++i)
	if (V(i) > max)
	  max = V(i);
      return max;
    }

  template<class T>
    double applyFilter(const usVolume<T> &Src, const usVolume<double> F,
		       unsigned int x0, unsigned int y0, unsigned int z0)
    {
      unsigned int s = F.getDimX();
      int m = s / 2;
      double v = 0.0;
      if ((m < x0) && (x0 < Src.getDimX() - m)
	  && (m < y0) && (y0 < Src.getDimY() - m)
	  && (m < z0) && (z0 < Src.getDimZ() - m))
	for (unsigned int z = 0; z < s; ++z)
	  for (unsigned int y = 0; y < s; ++y)
	    for (unsigned int x = 0; x < s; ++x)
	      v += F(x, y, z) * Src(x0 + x - m, y0 + y - m, z0 + z - m);
      return v;
    }

  template<class T>
    void applyFilter(const usVolume<T> &Src, usVolume<double> &Dst,
		     const usVolume<double> F)
    {
      unsigned int s = F.getDimX() / 2;
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      Dst.initData(0.0);
      for (unsigned int z = s; z < dimz - s; ++z)
	for (unsigned int y = s; y < dimy - s; ++y)
	  for (unsigned int x = s; x < dimx - s; ++x)
	    Dst(x, y, z, applyFilter(Src, F, x, y, z));
    }

  template<class T>
    double derivativeX(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x+1, y, z) - V(x-1, y, z)) / 2.0;
    }

  template<class T>
    double derivativeY(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x, y+1, z) - V(x, y-1, z)) / 2.0;
    }

  template<class T>
    double derivativeZ(const usVolume<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x, y, z+1) - V(x, y, z-1)) / 2.0;
    }

  template<class T>
    void derivativeX_2(const usVolume<T> &Src, usVolume<double> &Dst,
		       unsigned int filter_size, double sigma)
    {
      usVolume<double> F = getDerivativeFilterX(sigma, filter_size);
      applyFilter(Src, Dst, F);
    }

  template<class T>
    void derivativeX(const usVolume<T> &Src, usVolume<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int z = 0; z < dimz; ++z) {
	for (unsigned int y = 0; y < dimy; ++y) {
	  Dst(0, y, z, 0.0);
	  for (unsigned int x = 1; x < dimx-1; ++x)
	    Dst(x, y, z, derivativeX(Src, x, y, z));
	  Dst(dimx-1, y, z, 0.0);
	}
      }
    }

  template<class T>
    void derivativeY(const usVolume<T> &Src, usVolume<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int z = 0; z < dimz; ++z) {
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, 0, z, 0.0);
	for (unsigned int y = 1; y < dimy-1; ++y)
	  for (unsigned int x = 0; x < dimx; ++x)
	    Dst(x, y, z, derivativeY(Src, x, y, z));
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, dimy-1, z, 0.0);
      }
    }

  template<class T>
    void derivativeZ(const usVolume<T> &Src, usVolume<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int y = 0; y < dimy; ++y)
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, y, 0, 0.0);
      for (unsigned int z = 1; z < dimz-1; ++z) {
	for (unsigned int y = 0; y < dimy; ++y) {
	  for (unsigned int x = 0; x < dimx; ++x)
	    Dst(x, y, z, derivativeZ(Src, x, y, z));
	}
      }
      for (unsigned int y = 0; y < dimy; ++y)
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, y, dimz-1, 0.0);
    }
  
  template<class T>
    void gradient(const usVolume<T> &Src, usVolume<vpColVector> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      usVolume<double> Gx, Gy, Gz;
      derivativeX(Src, Gx);
      derivativeY(Src, Gy);
      derivativeZ(Src, Gz);
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int i = 0; i < Src.getSize(); ++i) {
	vpColVector v(3);
	v[0] = Gx(i);
	v[1] = Gy(i);
	v[2] = Gz(i);
	Dst(i, v);
      }
    }

  template<class T>
    void hessian(const usVolume<T> &Src, usVolume<vpMatrix> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      usVolume<double> Gx, Gy, Gz, Gxx, Gxy, Gxz, Gyy, Gyz, Gzz;
      derivativeX(Src, Gx);
      derivativeY(Src, Gy);
      derivativeZ(Src, Gz);
      derivativeX(Gx, Gxx);
      derivativeX(Gy, Gxy);
      derivativeX(Gz, Gxz);
      derivativeY(Gy, Gyy);
      derivativeY(Gz, Gyz);
      derivativeZ(Gz, Gzz);
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int i = 0; i < Src.getSize(); ++i) {
	vpMatrix M(3, 3);
	M[0][0] = Gxx(i);
	M[0][1] = Gxy(i);
	M[0][2] = Gxz(i);
	M[1][0] = Gxy(i);
	M[1][1] = Gyy(i);
	M[1][2] = Gyz(i);
	M[2][0] = Gxz(i);
	M[2][1] = Gyz(i);
	M[2][2] = Gzz(i);
	Dst(i, M);
      }
    }

  template<class T>
    void frangi(const usVolume<T> &Src, usVolume<double> &Dst,
		double a, double b, double c)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      usVolume<vpMatrix> H;
      hessian(Src, H);
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int i = 0; i < Src.getSize(); ++i) {
	vpColVector evalues = H(i).eigenValues();
	
	// Sort eigenvalues
	if (vpMath::abs(evalues[0]) > vpMath::abs(evalues[1]))
	  std::swap(evalues[0], evalues[1]);
	if (vpMath::abs(evalues[1]) > vpMath::abs(evalues[2]))
	  std::swap(evalues[1], evalues[2]);
	if (vpMath::abs(evalues[0]) > vpMath::abs(evalues[1]))
	  std::swap(evalues[0], evalues[1]);

	double v;
	if ((evalues[1] >= 0.0) || (evalues[2] >= 0.0))
	  v = 0.0;
	else {
	  double Rb = vpMath::abs(evalues[0])
	    / sqrt(vpMath::abs(evalues[1] * evalues[2]));
	  
	  double Ra = vpMath::abs(evalues[1] / evalues[2]);
	  
	  double S = evalues.euclideanNorm();
	  
	  v = (1.0 - exp(- vpMath::sqr(Ra) / (2.0 * vpMath::sqr(a))))
	    * exp(- vpMath::sqr(Rb) / (2.0 * vpMath::sqr(b)))
	    * (1.0 - exp(- vpMath::sqr(S) / (2.0 * vpMath::sqr(c))));
	}
	Dst(i, v);
      }
    }
  
  template<class T1, class T2>
    void difference(const usVolume<T1> &Src1, const usVolume<T1> &Src2, usVolume<T2> &Dst)
  {
    unsigned int dimx = Src1.getDimX();
    unsigned int dimy = Src1.getDimY();
    unsigned int dimz = Src1.getDimZ();
    unsigned int n = Src1.getSize();
    Dst.resize(dimx, dimy, dimz);
    for (unsigned int i = 0; i < n; ++i) {
      Dst(i, Src1(i) - Src2(i));
    }
  }

  template<class T1, class T2>
    void absoluteDifference(const usVolume<T1> &Src1, const usVolume<T1> &Src2, usVolume<T2> &Dst)
  {
    unsigned int dimx = Src1.getDimX();
    unsigned int dimy = Src1.getDimY();
    unsigned int dimz = Src1.getDimZ();
    unsigned int n = Src1.getSize();
    Dst.resize(dimx, dimy, dimz);
    for (unsigned int i = 0; i < n; ++i) {
      Dst(i, abs(Src1(i) - Src2(i)));
    }
  }
}

#endif
