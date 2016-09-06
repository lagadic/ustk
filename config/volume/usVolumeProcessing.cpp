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

#include "usVolumeProcessing.h"

void usVolumeProcessing::norm(const usVolume<vpColVector> &Src, usVolume<double> &Dst)
{
  unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
  Dst.resize(dimx, dimy, dimz);
  for (unsigned int i = 0; i < Src.getSize(); ++i) {
    Dst(i, Src(i).euclideanNorm());
  }
}

usVolume<double> usVolumeProcessing::getDerivativeFilterX(double sigma,
							  unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterX(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double dgx, gy, gz;
  for (unsigned int z = 0; z < size; ++z) {
    gz = exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (sigma * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      gy = exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (sigma * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	dgx = - (x - m) * exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
	F(x, y, z, dgx * gy * gz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterY(double sigma,
							  unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterY(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double gx, dgy, gz;
  for (unsigned int z = 0; z < size; ++z) {
    gz = exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (sigma * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      dgy = - (y - m) * exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	gx = exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (sigma * sqrt(2.0 * M_PI));
	F(x, y, z, gx * dgy * gz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterZ(double sigma,
							  unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterZ(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double gx, gy, dgz;
  for (unsigned int z = 0; z < size; ++z) {
    dgz =  - (z - m) * exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      gy = exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (sigma * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	gx = exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (sigma * sqrt(2.0 * M_PI));
	F(x, y, z, gx * gy * dgz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterXX(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterXX(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double ddgx, gy, gz;
  for (unsigned int z = 0; z < size; ++z) {
    gz = exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (sigma * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      gy = exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (sigma * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	ddgx = (vpMath::sqr(x - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI))
		- 1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI)))
	  * exp(- vpMath::sqr(x - m) / (2.0 * sigma2));
	F(x, y, z, ddgx * gy * gz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterYY(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterYY(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double gx, ddgy, gz;
  for (unsigned int z = 0; z < size; ++z) {
    gz = exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (sigma * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      ddgy = (vpMath::sqr(y - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI))
	      - 1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI)))
	* exp(- vpMath::sqr(y - m) / (2.0 * sigma2));
      for (unsigned int x = 0; x < size; ++x) {
	gx = exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (sigma * sqrt(2.0 * M_PI));
	F(x, y, z, gx * ddgy * gz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterZZ(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterZZ(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double gx, gy, ddgz;
  for (unsigned int z = 0; z < size; ++z) {
    ddgz = (vpMath::sqr(z - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI))
	    - 1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI)))
      * exp(- vpMath::sqr(z - m) / (2.0 * sigma2));
    for (unsigned int y = 0; y < size; ++y) {
      gy = exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (sigma * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	gx = exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (sigma * sqrt(2.0 * M_PI));
	F(x, y, z, gx * gy * ddgz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterXY(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterXY(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double dgx, dgy, gz;
  for (unsigned int z = 0; z < size; ++z) {
    gz = exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (sigma * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      dgy = - (y - m) * exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	dgx = - (x - m) * exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
	F(x, y, z, dgx * dgy * gz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterXZ(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterXZ(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double dgx, gy, dgz;
  for (unsigned int z = 0; z < size; ++z) {
    dgz = - (z - m) * exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      gy = exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (sigma * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	dgx = - (x - m) * exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
	F(x, y, z, dgx * gy * dgz);
      }
    }
  }
  return F;
}

usVolume<double> usVolumeProcessing::getDerivativeFilterYZ(double sigma,
							   unsigned int size)
{
  if ((size % 2) != 1) {
    std::cerr << "Error: In usVolumeProcessing::getDerivativeFilterXZ(): "
	      << "Bad filter size." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  double sigma2 = vpMath::sqr(sigma);
  int m = (static_cast<int>(size) - 1) / 2;
  usVolume<double> F;
  F.resize(size, size, size);
  double gx, dgy, dgz;
  for (unsigned int z = 0; z < size; ++z) {
    dgz = - (z - m) * exp(- vpMath::sqr(z - m) / (2.0 * sigma2))
      / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (unsigned int y = 0; y < size; ++y) {
      dgy = - (y - m) * exp(- vpMath::sqr(y - m) / (2.0 * sigma2))
	/ (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (unsigned int x = 0; x < size; ++x) {
	gx = exp(- vpMath::sqr(x - m) / (2.0 * sigma2))
	  / (sigma * sqrt(2.0 * M_PI));
	F(x, y, z, gx * dgy * dgz);
      }
    }
  }
  return F;
}

void usVolumeProcessing::computeBarycenter(const usVolume<unsigned char> &V,
					   double &xc, double &yc, double &zc)
{
  unsigned int dimx(V.getDimX());
  unsigned int dimy(V.getDimY());
  unsigned int dimz(V.getDimZ());
  double V_sum = 0.0;
  xc = 0.0;
  yc = 0.0;
  zc = 0.0;
  for (unsigned int x = 0; x < dimx; ++x)
    for (unsigned int y = 0; y < dimy; ++y)
      for (unsigned int z = 0; z < dimz; ++z) {
	xc += x * V(x, y, z);
	yc += y * V(x, y, z);
	zc += z * V(x, y, z);
	V_sum += V(x, y, z);
      }
  xc /= V_sum;
  yc /= V_sum;
  zc /= V_sum;
}
