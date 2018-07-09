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

/**
 * @file usVolumeProcessing.h
 * @brief Volume processing.
 */

#ifndef __usVolumeProcessing_h_
#define __usVolumeProcessing_h_

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
#include <type_traits>
#endif

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#include <visp3/ustk_core/usImage3D.h>

/**
 * @class usVolumeProcessing
 * @brief Processing tools (derivative, filtering...) for the usImage3D class.
 * @ingroup module_ustk_volume_processing
 */
class VISP_EXPORT usVolumeProcessing
{
public:
  template <class Type1, class Type2>
  static void absoluteDifference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst);

  template <class Type1, class Type2>
  static double applyFilter(const usImage3D<Type1> &src, const usImage3D<Type2> &filter, unsigned int i, unsigned int j,
                            unsigned int k);

  template <class Type1, class Type2>
  static void applyFilter(const usImage3D<Type1> &src, usImage3D<Type2> &dst, const usImage3D<double> &filter);

  template <class Type> static void computeBarycenter(const usImage3D<Type> &V, double &ic, double &jc, double &kc);

  template <class Type1, class Type2> static void derivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst);

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  template <class Type1, class Type2 = typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type>
  static Type2 derivativeI(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
#else
  template <class Type>
  static double derivativeI(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k);
#endif

  template <class Type1, class Type2> static void derivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst);

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  template <class Type1, class Type2 = typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type>
  static Type2 derivativeJ(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
#else
  template <class Type>
  static double derivativeJ(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k);
#endif

  template <class Type1, class Type2> static void derivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst);

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  template <class Type1, class Type2 = typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type>
  static Type2 derivativeK(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
#else
  template <class Type>
  static double derivativeK(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k);
#endif

  template <class Type1, class Type2>
  static void difference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst);

  template <class Type>
  static void frangi(const usImage3D<Type> &src, usImage3D<double> &dst, double a, double b, double c);

  template <class Type1, class Type2>
  static void gaussianDerivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                  unsigned int filter_size);

  template <class Type1, class Type2>
  static void gaussianDerivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                  unsigned int filter_size);

  template <class Type1, class Type2>
  static void gaussianDerivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                  unsigned int filter_size);

  static usImage3D<double> generateGaussianDerivativeFilterI(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterII(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterIJ(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterIK(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterJ(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterJJ(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterJK(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterK(double sigma, int size);

  static usImage3D<double> generateGaussianDerivativeFilterKK(double sigma, int size);

  template <class Type> static void gradient(const usImage3D<Type> &src, usImage3D<vpColVector> &dst);

  template <class Type> static void hessian(const usImage3D<Type> &src, usImage3D<vpMatrix> &dst);

  template <class Type> static Type max(const usImage3D<Type> &V);

  template <class Type> static Type min(const usImage3D<Type> &V);

  static void norm(const usImage3D<vpColVector> &src, usImage3D<double> &dst);
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
 * Get the max value of the volume.
 * @param V The volume to compute.
 * @return The maximum value of the volume.
 */
template <class Type> Type usVolumeProcessing::max(const usImage3D<Type> &V)
{
  const Type *p = V.getConstData();
  Type max = *p;
  for (unsigned int i = 1; i < V.getSize(); i++) {
    Type val = *p;
    if (val > max)
      max = val;
    p++;
  }
  return max;
}

/**
 * Get the min value of the volume.
 * @param V The volume to compute.
 * @return The minimum value of the volume.
 */
template <class Type> Type usVolumeProcessing::min(const usImage3D<Type> &V)
{
  const Type *p = V.getConstData();
  Type min = *p;
  for (unsigned int i = 1; i < V.getSize(); i++) {
    Type val = *p;
    if (val < min)
      min = val;
    p++;
  }
  return min;
}

/**
 * Apply a 3D filter to a voxel.
 * @param src The volume to filter.
 * @param filter The filter kernel.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return the value of the volxel filtered.
 */
template <class Type1, class Type2>
double usVolumeProcessing::applyFilter(const usImage3D<Type1> &src, const usImage3D<Type2> &filter, unsigned int i,
                                       unsigned int j, unsigned int k)
{
  unsigned int s_i = filter.getHeight();
  unsigned int s_j = filter.getWidth();
  unsigned int s_k = filter.getNumberOfFrames();
  unsigned int m_i = s_i / 2;
  unsigned int m_j = s_j / 2;
  unsigned int m_k = s_k / 2;
  Type2 v = Type2();

  if ((m_i < i) && (i < src.getHeight() - m_i) && (m_j < j) && (j < src.getWidth() - m_j) && (m_k < k) &&
      (k < src.getNumberOfFrames() - m_k)) {
    for (unsigned int k_it = 0; k_it < s_k; k_it++)
      for (unsigned int j_it = 0; j_it < s_j; j_it++)
        for (unsigned int i_it = 0; i_it < s_i; i_it++)
          v += filter(i_it, j_it, k_it) * src(i + i_it - m_i, j + j_it - m_j, k + k_it - m_k);
  }

  return v;
}

/**
 * Apply a filter to a volume.
 * @param [in] src The volume to filter.
 * @param [out] dst The volume filtered.
 * @param [in] filter The filter kernel.
 */
template <class Type1, class Type2>
void usVolumeProcessing::applyFilter(const usImage3D<Type1> &src, usImage3D<Type2> &dst,
                                     const usImage3D<double> &filter)
{
  unsigned int s_i = filter.getHeight();
  unsigned int s_j = filter.getWidth();
  unsigned int s_k = filter.getNumberOfFrames();

  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  dst.resize(height, width, nbFrames);
  dst.initData(0 * applyFilter(src, filter, s_i / 2, s_j / 2, s_k / 2));
  for (unsigned int k = s_k; k < nbFrames - s_k; k++)
    for (unsigned int j = s_j; j < width - s_j; j++)
      for (unsigned int i = s_i; i < height - s_i; i++)
        dst(i, j, k, applyFilter(src, filter, i, j, k));
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
/**
 * Apply a derivative filter along the i-axis (height) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
Type2 usVolumeProcessing::derivativeI(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((Type2)V(i + 1, j, k) - (Type2)V(i - 1, j, k)) / 2.0;
}

/**
 * Apply a derivative filter along the j-axis (width) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
Type2 usVolumeProcessing::derivativeJ(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((Type2)V(i, j + 1, k) - (Type2)V(i, j - 1, k)) / 2.0;
}

/**
 * Apply a derivative filter along the k-axis (3rd dimension) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
Type2 usVolumeProcessing::derivativeK(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((Type2)V(i, j, k + 1) - (Type2)V(i, j, k - 1)) / 2.0;
}
#else
/**
 * Apply a derivative filter along the i-axis (height) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type>
double usVolumeProcessing::derivativeI(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((double)V(i + 1, j, k) - (double)V(i - 1, j, k)) / 2.0;
}

/**
 * Apply a derivative filter along the j-axis (width) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type>
double usVolumeProcessing::derivativeJ(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((double)V(i, j + 1, k) - (double)V(i, j - 1, k)) / 2.0;
}

/**
 * Apply a derivative filter along the k-axis (3rd dimension) to a voxel.
 * @param V The volume to derivate.
 * @param i Index on i-axis of the voxel to filter.
 * @param j Index on j-axis of the voxel to filter.
 * @param k Index on k-axis of the voxel to filter.
 * @return The derivate output volume.
 */
template <class Type>
double usVolumeProcessing::derivativeK(const usImage3D<Type> &V, unsigned int i, unsigned int j, unsigned int k)
{
  return ((double)V(i, j, k + 1) - (double)V(i, j, k - 1)) / 2.0;
}
#endif

/**
 * Compute the gaussian filtered i-derivative (height) of a volume.
 * @param src The volume to filter.
 * @param dst The volume filtered.
 * @param sigma Gaussian filter standard deviation.
 * @param filter_size Size of the gaussian filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
void usVolumeProcessing::gaussianDerivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                             unsigned int filter_size)
{
  usImage3D<double> filter = generateGaussianDerivativeFilterI(sigma, filter_size);
  applyFilter(src, dst, filter);
}

/**
 * Compute the gaussian filtered j-derivative (width) of a volume.
 * @param src The volume to filter.
 * @param dst The volume filtered.
 * @param sigma Gaussian filter standard deviation.
 * @param filter_size Size of the gaussian filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
void usVolumeProcessing::gaussianDerivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                             unsigned int filter_size)
{
  usImage3D<double> filter = generateGaussianDerivativeFilterJ(sigma, filter_size);
  applyFilter(src, dst, filter);
}

/**
 * Compute the gaussian filtered k-derivative (3rd dimension) of a volume.
 * @param src The volume to filter.
 * @param dst The volume filtered.
 * @param sigma Gaussian filter standard deviation.
 * @param filter_size Size of the gaussian filter.
 * @return The derivate output volume.
 */
template <class Type1, class Type2>
void usVolumeProcessing::gaussianDerivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma,
                                             unsigned int filter_size)
{
  usImage3D<double> filter = generateGaussianDerivativeFilterK(sigma, filter_size);
  applyFilter(src, dst, filter);
}

/**
 * Apply a derivative filter along the i-axis (height) to a volume.
 * @param src The volume to derivate.
 * @param dst The volume derivated.
 */
template <class Type1, class Type2>
void usVolumeProcessing::derivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  dst.resize(height, width, nbFrames);
  Type2 zero = 0 * derivativeI(src, 1, 1, 1);
  // Access in order k-j-i for performance
  for (unsigned int k = 0; k < nbFrames; k++)
    for (unsigned int j = 0; j < width; j++) {
      dst(0, j, k, zero);
      for (unsigned int i = 1; i < height - 1; i++)
        dst(i, j, k, derivativeI(src, i, j, k));
      dst(height - 1, j, k, zero);
    }
}

/**
 * Apply a derivative filter along the j-axis (width) to a volume.
 * @param src The volume to derivate.
 * @param dst The volume derivated.
 */
template <class Type1, class Type2>
void usVolumeProcessing::derivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  dst.resize(height, width, nbFrames);
  Type2 zero = 0 * derivativeJ(src, 1, 1, 1);
  // Access in order k-j-i for performance
  for (unsigned int k = 0; k < nbFrames; k++) {
    for (unsigned int j = 1; j < width - 1; j++)
      for (unsigned int i = 0; i < height; i++)
        dst(i, j, k, derivativeJ(src, i, j, k));

    for (unsigned int i = 0; i < height; i++) {
      dst(i, width - 1, k, zero);
      dst(i, 0, k, zero);
    }
  }
}

/**
 * Apply a derivative filter along the k-axis (3rd dimension) to a volume.
 * @param src The volume to derivate.
 * @param dst The volume derivated.
 */
template <class Type1, class Type2>
void usVolumeProcessing::derivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  dst.resize(height, width, nbFrames);
  Type2 zero = 0 * derivativeK(src, 1, 1, 1);
  // Access in order k-j-i for performance
  for (unsigned int k = 1; k < nbFrames - 1; k++)
    for (unsigned int j = 0; j < width; j++)
      for (unsigned int i = 0; i < height; i++)
        dst(i, j, k, derivativeK(src, i, j, k));

  for (unsigned int j = 0; j < width; j++)
    for (unsigned int i = 0; i < height; i++) {
      dst(i, j, 0, zero);
      dst(i, j, nbFrames - 1, zero);
    }
}

/**
 * Compute the volume gradient.
 * @param src The volume to filter.
 * @param dst The volume filtered.
 */
template <class Type> void usVolumeProcessing::gradient(const usImage3D<Type> &src, usImage3D<vpColVector> &dst)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  usImage3D<double> Gi, Gj, Gk;
  derivativeI(src, Gi);
  derivativeJ(src, Gj);
  derivativeK(src, Gk);
  dst.resize(height, width, nbFrames);
  for (unsigned int i = 0; i < src.getSize(); i++) {
    vpColVector v(3);
    v[0] = Gi.getConstData()[i];
    v[1] = Gj.getConstData()[i];
    v[2] = Gk.getConstData()[i];
    dst.getData()[i] = v;
  }
}

/**
 * Compute the volume hessian.
 * @param src The volume to filter.
 * @param dst The volume filtered.
 */
template <class Type> void usVolumeProcessing::hessian(const usImage3D<Type> &src, usImage3D<vpMatrix> &dst)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  usImage3D<double> Gi, Gj, Gk, Gii, Gij, Gik, Gjj, Gjk, Gkk;
  derivativeI(src, Gi);
  derivativeJ(src, Gj);
  derivativeK(src, Gk);
  derivativeI(Gi, Gii);
  derivativeI(Gj, Gij);
  derivativeI(Gk, Gik);
  derivativeJ(Gj, Gjj);
  derivativeJ(Gk, Gjk);
  derivativeK(Gk, Gkk);
  dst.resize(height, width, nbFrames);
  for (unsigned int i = 0; i < src.getSize(); i++) {
    vpMatrix M(3, 3);
    M[0][0] = Gii.getConstData()[i];
    M[0][1] = Gij.getConstData()[i];
    M[0][2] = Gik.getConstData()[i];
    M[1][0] = Gij.getConstData()[i];
    M[1][1] = Gjj.getConstData()[i];
    M[1][2] = Gjk.getConstData()[i];
    M[2][0] = Gik.getConstData()[i];
    M[2][1] = Gjk.getConstData()[i];
    M[2][2] = Gkk.getConstData()[i];
    dst.getData()[i] = M;
  }
}

/**
 * Compute the Frangi's vesselness.
 * Here is the frangi vesselness formula, from Frangi et al. (1998):
 *   \f[
  V_{0}(s) =  (1 - exp(- \frac{- R_{A}^{2}}{2α^{2}}))exp(- \frac{- R_{B}^{2}}{2β^{2}})(1 - exp(- \frac{S^{2}}{2c^{2}}))
  \f]
 * @warning This method needs GSL thirdparty to work. You can prevent errors by testing VISP_HAVE_GSL define.
 * @param src The volume to computed.
 * @param dst The volume filtered.
 * @param a Corresponds to \f$ α \f$ parameter in formula above.
 * @param b Corresponds to \f$ β \f$ parameter in formula above.
 * @param c Corresponds to \f$ c \f$ parameter in formula above.
 */
template <class Type>
void usVolumeProcessing::frangi(const usImage3D<Type> &src, usImage3D<double> &dst, double a, double b, double c)
{
  unsigned int height = src.getHeight();
  unsigned int width = src.getWidth();
  unsigned int nbFrames = src.getNumberOfFrames();
  usImage3D<vpMatrix> H;
  hessian(src, H);
  dst.resize(height, width, nbFrames);
  for (unsigned int i = 0; i < src.getSize(); i++) {
    vpColVector evalues = H.getConstData()[i].eigenValues();

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
      double Rb = vpMath::abs(evalues[0]) / sqrt(vpMath::abs(evalues[1] * evalues[2]));
      double Ra = vpMath::abs(evalues[1] / evalues[2]);
      double S = evalues.euclideanNorm();

      v = (1.0 - exp(-vpMath::sqr(Ra) / (2.0 * vpMath::sqr(a)))) * exp(-vpMath::sqr(Rb) / (2.0 * vpMath::sqr(b))) *
          (1.0 - exp(-vpMath::sqr(S) / (2.0 * vpMath::sqr(c))));
    }
    dst.getData()[i] = v;
  }
}

/**
 * Compute the volume difference: dst = src1 - src2.
 * @param src1 The first volume.
 * @param src2 The second volume.
 * @param dst The output volume (difference).
 */
template <class Type1, class Type2>
void usVolumeProcessing::difference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst)
{
  unsigned int height = src1.getHeight();
  unsigned int width = src1.getWidth();
  unsigned int nbFrames = src1.getNumberOfFrames();
  if (height != src2.getHeight() || width != src2.getWidth() || nbFrames != src2.getNumberOfFrames())
    throw vpException(vpException::dimensionError, "usVolumeProcessing::difference: mismatched volumes dimensions");
  unsigned int n = src1.getSize();
  dst.resize(height, width, nbFrames);
  for (unsigned int i = 0; i < n; i++)
    dst.getData()[i] = src1.getConstData()[i] - src2.getConstData()[i];
}

/**
 * Compute the volume absolute difference: dst = |src1 - src2|.
 * @param src1 The first volume.
 * @param src2 The second volume.
 * @param dst The output volume (difference).
 */
template <class Type1, class Type2>
void usVolumeProcessing::absoluteDifference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2,
                                            usImage3D<Type2> &dst)
{
  unsigned int height = src1.getHeight();
  unsigned int width = src1.getWidth();
  unsigned int nbFrames = src1.getNumberOfFrames();
  if (height != src2.getHeight() || width != src2.getWidth() || nbFrames != src2.getNumberOfFrames())
    throw vpException(vpException::dimensionError,
                      "usVolumeProcessing::absoluteDifference: mismatched volumes dimensions");
  unsigned int n = src1.getSize();
  dst.resize(height, width, nbFrames);
  for (unsigned int i = 0; i < n; i++)
    dst.getData()[i] = vpMath::abs(src1.getConstData()[i] - src2.getConstData()[i]);
}

/**
 * Compute the photometric barycenter of the volume.
 * @param [in] V Volume to compute.
 * @param [out] ic I-axis index of the barycenter.
 * @param [out] jc J-axis index of the barycenter.
 * @param [out] kc K-axis index of the barycenter.
 */
template <class Type>
void usVolumeProcessing::computeBarycenter(const usImage3D<Type> &V, double &ic, double &jc, double &kc)
{
  unsigned int height = V.getHeight();
  unsigned int width = V.getWidth();
  unsigned int nbFrames = V.getNumberOfFrames();
  Type V_sum = 0 * V(0, 0, 0);
  double i_c = 0.0;
  double j_c = 0.0;
  double k_c = 0.0;
  for (unsigned int k = 0; k < nbFrames; k++)
    for (unsigned int j = 0; j < width; j++)
      for (unsigned int i = 0; i < height; i++) {
        Type val = V(i, j, k);
        i_c += i * val;
        j_c += j * val;
        k_c += k * val;
        V_sum += val;
      }
  ic = i_c / V_sum;
  jc = j_c / V_sum;
  kc = k_c / V_sum;
}

#endif // __usVolumeProcessing_h_
