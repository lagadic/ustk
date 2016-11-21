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
 * @file usImageMathematics.h
 * @brief Image processing tools.
 * @author Pierre Chatelain
 */

#ifndef US_IMAGE_MATHEMATICS
#define US_IMAGE_MATHEMATICS

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usRectangle.h>

/**
 * @namespace usImageMathematics
 * @brief Image processing tools.
 * @ingroup module_ustk_core
 *
 * This class implements processing tools for ultrasound images.
 */
namespace usImageMathematics {

enum InterpolationType {
  NEAREST, BILINEAR, BICUBIC
};

/**
* Return the minimum value of the image.
*/
template <class T>
T min(const vpImage<T>& I);

/**
* Return the maximum value of the image.
*/
template <class T>
T max(const vpImage<T>& I);

/**
* Compute the image mean.
*/
double mean(const vpImage<double>& I);

/**
* Compute the sum of image intensities.
*/
double sum(const vpImage<double>& I);

/**
* Compute the sum of image intensities.
*/
double sum(const vpImage<unsigned char>& I);

/**
* Compute the absolute difference image from two images.
*/
void absoluteDiff(const vpImage<double>& I, const vpImage<double>& J,
                  vpImage<double>& D);

/**
* Compute the normalized correlation between two images.
*/
double normalizedCorrelation(const vpImage<double>& I, const vpImage<double>& J);

/**
* Normalize the image intensities.
*/
void normalize(vpImage<double>& I);

/**
* Compute the column-wise mean intensities.
*/
void computeColumnMean(const vpImage<double>& I, vpColVector& V);

/**
* Compute the column-wise mean intensities.
*/
void computeColumnMean(const vpImage<unsigned char>& I, vpColVector& V,
                       const bool &rescale = true);

/**
* Compute the mean value of an image within a mask.
*/
double computeRegionMean(const vpImage<unsigned char>& I, const vpImage<unsigned char>& M);

/**
* Compute the photometric moment of the image at order (p,q).
*/
double computePhotometricMoment(const vpImage<unsigned char> &I, int p, int q);

/**
* Compute the photometric moment of the object at order (p,q).
*/
double computePhotometricMoment(const vpImage<unsigned char> &I, const vpImage<unsigned char> &M,
                                int p, int q);

/**
* Compute the image's photometric barycenter (first-order moment).
*/
void computeBarycenter(const vpImage<unsigned char> &I, double &xc, double &yc);

/**
* Compute the object's photometric barycenter (first-order moment).
*/
void computeBarycenter(const vpImage<unsigned char> &I, const vpImage<unsigned char> &M,
                       double &xc, double &yc);

/**
* Get the interpolated value at a given location.
*/
double VISP_EXPORT interpolate(const vpImage<unsigned char> &I, double x, double y, InterpolationType it);

/**
* Re-sample an image.
*/
void resample(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, InterpolationType it);

/**
* Extract a rectangular region from an image.
*/
void VISP_EXPORT extract(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, const usRectangle &r);

/**
* Extract a rectangular region from an image.
*/
void VISP_EXPORT extract(const vpImage<unsigned char> &Src, vpImage<double> &Dst, const usRectangle &r);
}

// Template implementations
namespace usImageMathematics {

template <class T>
T min(const vpImage<T>& I)
{
  unsigned int size = I.getSize();
  T min = I.bitmap[0];
  for (unsigned int i = 1; i < size; ++i)
    if (I.bitmap[i] < min) min = I.bitmap[i];

  return min;
}

template <class T>
T max(const vpImage<T>& I)
{
  unsigned int size = I.getSize();
  T max = I.bitmap[0];
  for (unsigned int i = 1; i < size; ++i)
    if (I.bitmap[i] > max) max = I.bitmap[i];

  return max;
}
}

#endif // US_IMAGE_MATHEMATICS

