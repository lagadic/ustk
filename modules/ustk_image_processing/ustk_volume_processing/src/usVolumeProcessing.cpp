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
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/core/vpException.h>
#include <visp3/ustk_volume_processing/usVolumeProcessing.h>

/**
 * Compute the norm of a vector image.
 * @param src Input volume.
 * @param dst Output volume.
 */
void usVolumeProcessing::norm(const usImage3D<vpColVector> &src, usImage3D<double> &dst)
{
  dst.resize(src.getHeight(), src.getWidth(), src.getNumberOfFrames());

  for (unsigned int i = 0; i < src.getSize(); i++)
    dst.getData()[i] = src.getConstData()[i].euclideanNorm();
}

/**
 * Generate a Gaussian filtered derivative filter along the j-axis (width).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterI(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterI(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double dgi, gj, gk;
  for (int k = 0; k < size; k++) {
    gk = exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      gj = exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        dgi = -(i - m) * exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
        filter(i, j, k, dgi * gj * gk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered derivative filter along the j-axis (width).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterJ(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterJ(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double gi, dgj, gk;
  for (int k = 0; k < size; k++) {
    gk = exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      dgj = -(j - m) * exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        gi = exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
        filter(i, j, k, gi * dgj * gk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered derivative filter along the k-axis (3rd dimension).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterK(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterK(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double gi, gj, dgk;
  for (int k = 0; k < size; k++) {
    dgk = -(k - m) * exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      gj = exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        gi = exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
        filter(i, j, k, gi * gj * dgk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the i-axis (height).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterII(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterII(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double ddgi, gj, gk;
  for (int k = 0; k < size; k++) {
    gk = exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      gj = exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        ddgi = (vpMath::sqr(i - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI)) -
                1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI))) *
               exp(-vpMath::sqr(i - m) / (2.0 * sigma2));
        filter(i, j, k, ddgi * gj * gk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the j-axis (width).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterJJ(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterJJ(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double gi, ddgj, gk;
  for (int k = 0; k < size; k++) {
    gk = exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      ddgj = (vpMath::sqr(j - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI)) -
              1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI))) *
             exp(-vpMath::sqr(j - m) / (2.0 * sigma2));
      for (int i = 0; i < size; i++) {
        gi = exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
        filter(i, j, k, gi * ddgj * gk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the k-axis (3rd dimension).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterKK(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterKK(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double gi, gj, ddgk;
  for (int k = 0; k < size; k++) {
    ddgk = (vpMath::sqr(k - m) / (4.0 * vpMath::sqr(sigma2) * sqrt(2.0 * M_PI)) -
            1.0 / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI))) *
           exp(-vpMath::sqr(k - m) / (2.0 * sigma2));
    for (int j = 0; j < size; j++) {
      gj = exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        gi = exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
        filter(i, j, k, gi * gj * ddgk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the ij-axis (height/width diagonal).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterIJ(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterIJ(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double dgi, dgj, gk;
  for (int k = 0; k < size; k++) {
    gk = exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      dgj = -(j - m) * exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        dgi = -(i - m) * exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
        filter(i, j, k, dgi * dgj * gk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the ik-axis (height/3rd dimension diagonal).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterIK(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterIK(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double dgi, gj, dgk;
  for (int k = 0; k < size; k++) {
    dgk = -(k - m) * exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      gj = exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        dgi = -(i - m) * exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
        filter(i, j, k, dgi * gj * dgk);
      }
    }
  }
  return filter;
}

/**
 * Generate a Gaussian filtered second derivative filter along the jk-axis (width/3rd dimension diagonal).
 * @param sigma Gaussian filter standard deviation.
 * @param size Size of the gaussian filter.
 */
usImage3D<double> usVolumeProcessing::generateGaussianDerivativeFilterJK(double sigma, int size)
{
  if (size <= 0 || (size % 2) != 1) {
    throw vpException(
        vpException::badValue,
        "usVolumeProcessing::generateGaussianDerivativeFilterJK(): Bad filter size, should be positive and odd");
  }

  double sigma2 = vpMath::sqr(sigma);
  int m = (size - 1) / 2;
  usImage3D<double> filter(size, size, size);

  double gi, dgj, dgk;
  for (int k = 0; k < size; k++) {
    dgk = -(k - m) * exp(-vpMath::sqr(k - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
    for (int j = 0; j < size; j++) {
      dgj = -(j - m) * exp(-vpMath::sqr(j - m) / (2.0 * sigma2)) / (2.0 * sigma * sigma2 * sqrt(2.0 * M_PI));
      for (int i = 0; i < size; i++) {
        gi = exp(-vpMath::sqr(i - m) / (2.0 * sigma2)) / (sigma * sqrt(2.0 * M_PI));
        filter(i, j, k, gi * dgj * dgk);
      }
    }
  }
  return filter;
}
