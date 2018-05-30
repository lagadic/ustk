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

#include <type_traits>

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>

#include <visp3/ustk_core/usImage3D.h>

/**
 * @class usVolumeProcessing
 * @brief Processing tools (derivative, filtering...) for the usImage3D class.
 * @ingroup module_ustk_volume_processing
 *
 */
class VISP_EXPORT usVolumeProcessing
{
public:
  /**
   * Get the max value.
   */
  template<class Type>
  static Type max(const usImage3D<Type> &V);
  
  /**
   * Get the min value.
   */
  template<class Type>
  static Type min(const usImage3D<Type> &V);

  /**
   * Apply a filter to a voxel.
   */
  template<class Type1, class Type2>
  static double applyFilter(const usImage3D<Type1> &src, const usImage3D<Type2> &filter, unsigned int i, unsigned int j, unsigned int k);

  /**
   * Apply a filter to a volume.
   */
  template<class Type1, class Type2>
  static void applyFilter(const usImage3D<Type1> &src, usImage3D<Type2> &dst, const usImage3D<double> &filter);

  /**
   * Apply a derivative filter along the i-axis (height) to a voxel.
   */
  template<class Type1, class Type2= typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type >
  static Type2 derivativeI(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
  
  /**
   * Apply a derivative filter along the j-axis (width) to a voxel.
   */
  template<class Type1, class Type2= typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type >
  static Type2 derivativeJ(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
  
  /**
   * Apply a derivative filter along the k-axis (3rd dimension) to a voxel.
   */
  template<class Type1, class Type2= typename std::conditional<std::is_arithmetic<Type1>::value, double, Type1>::type >
  static Type2 derivativeK(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k);
  
  /**
   * Compute the gaussian filtered i-derivative (height) of a volume
   */
  template<class Type1, class Type2>
  static void gaussianDerivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size);

  /**
   * Compute the gaussian filtered j-derivative (width) of a volume
   */
  template<class Type1, class Type2>
  static void gaussianDerivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size);
  
  /**
   * Compute the gaussian filtered k-derivative (3rd dimension) of a volume
   */
  template<class Type1, class Type2>
  static void gaussianDerivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size);

  /**
   * Apply a derivative filter along the i-axis (height) to a volume.
   */
  template<class Type1, class Type2>
  static void derivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst);
  
  /**
   * Apply a derivative filter along the j-axis (width) to a volume.
   */
  template<class Type1, class Type2>
  static void derivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst);
  
  /**
   * Apply a derivative filter along the k-axis (3rd dimension) to a volume.
   */
  template<class Type1, class Type2>
  static void derivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst);
  
  /**
   * Compute the volume gradient.
   */
  template<class Type>
  static void gradient(const usImage3D<Type> &src, usImage3D<vpColVector> &dst);

  /**
   * Compute the volume hessian.
   */
  template<class Type>
  static void hessian(const usImage3D<Type> &src, usImage3D<vpMatrix> &dst);

  /**
   * Compute the Frangi's vesselness.
   */
  template<class Type>
  static void frangi(const usImage3D<Type> &src, usImage3D<double> &dst, double a, double b, double c);

  /**
   * Compute the norm of a vector image.
   */
  static void norm(const usImage3D<vpColVector> &src, usImage3D<double> &dst);

  /**
   * Compute the volume difference.
   */
  template<class Type1, class Type2>
  static void difference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst);

  /**
   * Compute the volume absolute difference.
   */
  template<class Type1, class Type2>
  static void absoluteDifference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst);
  
  /**
   * Compute the photometric barycenter of the volume.
   */
  template<class Type>
  static void computeBarycenter(const usImage3D<Type> &V, double &ic, double &jc, double &kc);

  /**
   * Generate a Gaussian filtered derivative filter along the i-axis (height).
   */
  static usImage3D<double> generateGaussianDerivativeFilterI(double sigma, int size);

  /**
   * Generate a Gaussian filtered derivative filter along the j-axis (width).
   */
  static usImage3D<double> generateGaussianDerivativeFilterJ(double sigma, int size);

  /**
   * Generate a Gaussian filtered derivative filter along the k-axis (3rd dimension).
   */
  static usImage3D<double> generateGaussianDerivativeFilterK(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the i-axis (height).
   */
  static usImage3D<double> generateGaussianDerivativeFilterII(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the j-axis (width).
   */
  static usImage3D<double> generateGaussianDerivativeFilterJJ(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the k-axis (3rd dimension).
   */
  static usImage3D<double> generateGaussianDerivativeFilterKK(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the ij-axis (height/width diagonal).
   */
  static usImage3D<double> generateGaussianDerivativeFilterIJ(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the ik-axis (height/3rd dimension diagonal).
   */
  static usImage3D<double> generateGaussianDerivativeFilterIK(double sigma, int size);

  /**
   * Generate a Gaussian filtered second derivative filter along the jk-axis (width/3rd dimension diagonal).
   */
  static usImage3D<double> generateGaussianDerivativeFilterJK(double sigma, int size);

};

/****************************************************************************
* Template implementations.
****************************************************************************/

  template<class Type>
  Type usVolumeProcessing::max(const usImage3D<Type> &V)
  {
      const Type *p = V.getConstData();
      Type max = *p;
      for(unsigned int i=1 ; i<V.getSize() ; i++)
      {
          Type val = *p;
          if(val > max) max = val;
          p++;
      }
      return max;
  }
  
  template<class Type>
  Type usVolumeProcessing::min(const usImage3D<Type> &V)
  {
      const Type *p = V.getConstData();
      Type min = *p;
      for(unsigned int i=1 ; i<V.getSize() ; i++)
      {
          Type val = *p;
          if(val < min) min = val;
          p++;
      }
      return min;
  }

  template<class Type1, class Type2>
  double usVolumeProcessing::applyFilter(const usImage3D<Type1> &src, const usImage3D<Type2> &filter, unsigned int i, unsigned int j, unsigned int k)
  {
      unsigned int s_i = filter.getHeight();
      unsigned int s_j = filter.getWidth();
      unsigned int s_k = filter.getNumberOfFrames();
      unsigned int m_i = s_i / 2;
      unsigned int m_j = s_j / 2;
      unsigned int m_k = s_k / 2;
      Type2 v = Type2();
      
      if(   (m_i < i) && (i < src.getHeight() - m_i)
	     && (m_j < j) && (j < src.getWidth() - m_j)
	     && (m_k < k) && (k < src.getNumberOfFrames() - m_k))
      {
        for(unsigned int k_it=0 ; k_it<s_k ; k_it++)
          for(unsigned int j_it=0 ; j_it<s_j ; j_it++)
            for(unsigned int i_it=0 ; i_it<s_i ; i_it++)
              v += filter(i_it, j_it, k_it) * src(i + i_it - m_i, j + j_it - m_j, k + k_it - m_k);
      }
      
      return v;
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::applyFilter(const usImage3D<Type1> &src, usImage3D<Type2> &dst, const usImage3D<double> &filter)
  {
      unsigned int s_i = filter.getHeight();
      unsigned int s_j = filter.getWidth();
      unsigned int s_k = filter.getNumberOfFrames();

      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      dst.resize(height, width, nbFrames);
      dst.initData(0*applyFilter(src, filter, s_i/2, s_j/2, s_k/2));
      for(unsigned int k=s_k ; k<nbFrames-s_k ; k++)
        for(unsigned int j=s_j ; j<width-s_j ; j++)
	      for(unsigned int i=s_i ; i<height-s_i ; i++)
	        dst(i, j, k, applyFilter(src, filter, i, j, k));
  }

  template<class Type1, class Type2>
  Type2 usVolumeProcessing::derivativeI(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
  {
      return (V(i+1, j, k) - V(i-1, j, k)) / 2.0;
  }
  
  template<class Type1, class Type2>
  Type2 usVolumeProcessing::derivativeJ(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
  {
      return (V(i, j+1, k) - V(i, j-1, k)) / 2.0;
  }

  template<class Type1, class Type2>
  Type2 usVolumeProcessing::derivativeK(const usImage3D<Type1> &V, unsigned int i, unsigned int j, unsigned int k)
  {
      return (V(i, j, k+1) - V(i, j, k-1)) / 2.0;
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::gaussianDerivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size)
  {
      usImage3D<double> filter = generateGaussianDerivativeFilterI(sigma, filter_size);
      applyFilter(src, dst, filter);
  }
  
  template<class Type1, class Type2>
  void usVolumeProcessing::gaussianDerivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size)
  {
      usImage3D<double> filter = generateGaussianDerivativeFilterJ(sigma, filter_size);
      applyFilter(src, dst, filter);
  }
  
  template<class Type1, class Type2>
  void usVolumeProcessing::gaussianDerivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst, double sigma, unsigned int filter_size)
  {
      usImage3D<double> filter = generateGaussianDerivativeFilterK(sigma, filter_size);
      applyFilter(src, dst, filter);
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::derivativeI(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
  {
      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      dst.resize(height, width, nbFrames);
      Type2 zero = 0*derivativeI<Type1, Type2>(src, 1, 1, 1);
      // Access in order k-j-i for performance
      for(unsigned int k=0 ; k<nbFrames ; k++)
        for(unsigned int j=0 ; j<width ; j++)
        {
            dst(0, j, k, zero);
            for(unsigned int i=1 ; i<height-1 ; i++) dst(i, j, k, derivativeI<Type1, Type2>(src, i, j, k));
            dst(height-1, j, k, zero);
        }
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::derivativeJ(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
  {
      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      dst.resize(height, width, nbFrames);
      Type2 zero = 0*derivativeJ<Type1, Type2>(src, 1, 1, 1);
      // Access in order k-j-i for performance
      for(unsigned int k=0 ; k<nbFrames; k++)
      {
	      for(unsigned int j=1 ; j<width-1 ; j++)
            for(unsigned int i=0 ; i<height ; i++)
              dst(i, j, k, derivativeJ<Type1, Type2>(src, i, j, k));
          
          for(unsigned int i=0 ; i<height ; i++)
          {
              dst(i, width-1, k, zero);
              dst(i, 0, k, zero);
          }
      }
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::derivativeK(const usImage3D<Type1> &src, usImage3D<Type2> &dst)
  {
      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      dst.resize(height, width, nbFrames);
      Type2 zero = 0*derivativeK<Type1, Type2>(src, 1, 1, 1);
      // Access in order k-j-i for performance     
      for(unsigned int k=1 ; k<nbFrames-1 ; k++)
	    for(unsigned int j=0 ; j<width ; j++)
	      for(unsigned int i=0 ; i<height ; i++)
            dst(i, j, k, derivativeK<Type1, Type2>(src, i, j, k));
      
      for(unsigned int j=0 ; j<width ; j++)
	    for(unsigned int i=0 ; i<height ; i++)
        {
	      dst(i, j, 0, zero);
          dst(i, j, nbFrames-1, zero);
        }
  }
  
  template<class Type>
  void usVolumeProcessing::gradient(const usImage3D<Type> &src, usImage3D<vpColVector> &dst)
  {
      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      usImage3D<double> Gi, Gj, Gk;
      derivativeI(src, Gi);
      derivativeJ(src, Gj);
      derivativeK(src, Gk);
      dst.resize(height, width, nbFrames);
      for(unsigned int i=0 ; i<src.getSize() ; i++)
      {
          vpColVector v(3);
	      v[0] = Gi.getConstData()[i];
	      v[1] = Gj.getConstData()[i];
	      v[2] = Gk.getConstData()[i];
	      dst.getData()[i] = v;
      }
  }

  template<class Type>
  void usVolumeProcessing::hessian(const usImage3D<Type> &src, usImage3D<vpMatrix> &dst)
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
      for(unsigned int i=0 ; i<src.getSize() ; i++)
      {
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

  template<class Type>
  void usVolumeProcessing::frangi(const usImage3D<Type> &src, usImage3D<double> &dst, double a, double b, double c)
  {
      unsigned int height = src.getHeight();
      unsigned int width = src.getWidth();
      unsigned int nbFrames = src.getNumberOfFrames();
      usImage3D<vpMatrix> H;
      hessian(src, H);
      dst.resize(height, width, nbFrames);
      for(unsigned int i=0 ; i<src.getSize() ; i++)
      {
	      vpColVector evalues = H.getConstData()[i].eigenValues();
	
	      // Sort eigenvalues
	      if(vpMath::abs(evalues[0]) > vpMath::abs(evalues[1])) std::swap(evalues[0], evalues[1]);
	      if(vpMath::abs(evalues[1]) > vpMath::abs(evalues[2])) std::swap(evalues[1], evalues[2]);
	      if(vpMath::abs(evalues[0]) > vpMath::abs(evalues[1])) std::swap(evalues[0], evalues[1]);

	      double v;
	      if((evalues[1] >= 0.0) || (evalues[2] >= 0.0)) v = 0.0;
	      else
          {
	           double Rb = vpMath::abs(evalues[0]) / sqrt(vpMath::abs(evalues[1] * evalues[2]));
	           double Ra = vpMath::abs(evalues[1] / evalues[2]);
	           double S = evalues.euclideanNorm();
	  
	           v = (1.0 - exp(- vpMath::sqr(Ra) / (2.0 * vpMath::sqr(a))))
	              * exp(- vpMath::sqr(Rb) / (2.0 * vpMath::sqr(b)))
	              * (1.0 - exp(- vpMath::sqr(S) / (2.0 * vpMath::sqr(c))));
	      }
          dst.getData()[i] = v;
      }
  }
  
  template<class Type1, class Type2>
  void usVolumeProcessing::difference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst)
  {
      unsigned int height = src1.getHeight();
      unsigned int width = src1.getWidth();
      unsigned int nbFrames = src1.getNumberOfFrames();
      if(height != src2.getHeight() || width != src2.getWidth() || nbFrames != src2.getNumberOfFrames())
          throw vpException(vpException::dimensionError, "usVolumeProcessing::difference: mismatched volumes dimensions");
      unsigned int n = src1.getSize();
      dst.resize(height, width, nbFrames);
      for(unsigned int i=0 ; i<n ; i++) dst.getData()[i] = src1.getConstData()[i] - src2.getConstData()[i];
  }

  template<class Type1, class Type2>
  void usVolumeProcessing::absoluteDifference(const usImage3D<Type1> &src1, const usImage3D<Type1> &src2, usImage3D<Type2> &dst)
  {
      unsigned int height = src1.getHeight();
      unsigned int width = src1.getWidth();
      unsigned int nbFrames = src1.getNumberOfFrames();
      if(height != src2.getHeight() || width != src2.getWidth() || nbFrames != src2.getNumberOfFrames())
          throw vpException(vpException::dimensionError, "usVolumeProcessing::absoluteDifference: mismatched volumes dimensions");
      unsigned int n = src1.getSize();
      dst.resize(height, width, nbFrames);
      for(unsigned int i=0 ; i<n ; i++) dst.getData()[i] = vpMath::abs(src1.getConstData()[i] - src2.getConstData()[i]);
  }
  
  template<class Type>
  void usVolumeProcessing::computeBarycenter(const usImage3D<Type> &V, double &ic, double &jc, double &kc)
  {
      unsigned int height = V.getHeight();
      unsigned int width = V.getWidth();
      unsigned int nbFrames = V.getNumberOfFrames();
      Type V_sum = 0*V(0,0,0);
      double i_c = 0.0;
      double j_c = 0.0;
      double k_c = 0.0;
      for(unsigned int k=0 ; k<nbFrames; k++)
        for (unsigned int j=0 ; j<width ; j++)
          for (unsigned int i=0 ; i<height ; i++)
          {
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
