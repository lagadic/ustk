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
 *
 *****************************************************************************/

#include <visp3/ustk_core/usImageMathematics.h>

namespace usImageMathematics {

  double mean(const vpImage<double>& I) {
    double res = 0.0;
    unsigned int height = I.getHeight();
    unsigned int width = I.getWidth();
    if ((height == 0) || (width == 0))
      return 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
  res += I[i][j];
    res /= (height * width);
    return res;
  }

  double sum(const vpImage<double>& I) {
    double res = 0.0;
    unsigned int height = I.getHeight();
    unsigned int width = I.getWidth();
    if ((height == 0) || (width == 0))
      return 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
  res += I[i][j];
    return res;
  }

  double sum(const vpImage<unsigned char>& I) {
    double res = 0.0;
    unsigned int height = I.getHeight();
    unsigned int width = I.getWidth();
    if ((height == 0) || (width == 0))
      return 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
  res += static_cast<double>(I[i][j]);
    return res;
  }

  void absoluteDiff(const vpImage<double>& I, const vpImage<double>& J, vpImage<double>& D)
  {
    if ((I.getHeight() != J.getHeight()) || (I.getWidth() != J.getWidth())) {
      std::cerr << "Error: in usImageMathematics::absoluteDiff(): "
    << "image dimensions mismatch." << std::endl;
      exit(EXIT_FAILURE);
    }
    D.resize(I.getHeight(), I.getWidth());
    for (unsigned int i = 0; i < D.getHeight(); ++i)
      for (unsigned int j = 0; j < D.getWidth(); ++j)
  D(i, j, vpMath::abs(I(i, j) - J(i, j)));
  }

  double normalizedCorrelation(const vpImage<double>& I, const vpImage<double>& J)
  {
    if ((I.getHeight() != J.getHeight()) || (I.getWidth() != J.getWidth())) {
      std::cerr << "Error: in usImageMathematics::normalizedCorrelation(): "
    << "image dimensions mismatch." << std::endl;
      exit(EXIT_FAILURE);
    }

    double a = mean(I);
    double b = mean(J);

    double ab = 0.0;
    double a2 = 0.0;
    double b2 = 0.0;

    for (unsigned int i = 0; i < I.getHeight(); ++i)
      for (unsigned int j = 0; j < I.getWidth(); ++j)
  {
    ab += (I(i, j) - a) * (J(i, j) - b);
    a2 += vpMath::sqr(I(i, j) - a);
    b2 += vpMath::sqr(J(i, j) - b);
  }

    return ab / sqrt(a2 * b2);
  }

  void normalize(vpImage<double>& I)
  {
    double s = sum(I);
    for (unsigned int i = 0; i < I.getHeight(); ++i)
      for (unsigned int j = 0; j < I.getWidth(); ++j)
  I(i, j, I(i, j) / s);
  }

  void computeColumnMean(const vpImage<double>& I, vpColVector& V) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    V.resize(width);
    for (unsigned int j = 0; j < width; ++j)
      V[j] = 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
  V[j] += I[i][j];
    for (unsigned int j = 0; j < width; ++j)
      V[j] /= height;
  }

  void computeColumnMean(const vpImage<unsigned char>& I, vpColVector& V, const bool &rescale) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    V.resize(width);
    for (unsigned int j = 0; j < width; ++j)
      V[j] = 0.0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
  V[j] += I[i][j];
    for (unsigned int j = 0; j < width; ++j)
      if (rescale)
  V[j] /= (height * 255);
      else
  V[j] /= height;
  }

  double computeRegionMean(const vpImage<unsigned char>& I, const vpImage<unsigned char>& M) {
    unsigned int height(I.getHeight()), width(I.getWidth());

    if ((height != M.getHeight()) || (width != M.getWidth())) {
      std::cerr << "Error: in usImageMathematics::computeRegionMean(): "
    << "image dimensions mismatch." << std::endl;
      exit(EXIT_FAILURE);
    }

    double v = 0.0;
    unsigned int c = 0;
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j) {
  if (M(i,j)) {
    v += I[i][j];
    c++;
      }
    }

    if (c == 0)
      return 0.0;

    return v / c;
  }

  double computePhotometricMoment(const vpImage<unsigned char> &I, int p, int q) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    double m = 0.0;
    for (unsigned int x = 0; x < height; ++x)
      for (unsigned int y = 0; y < width; ++y) {
  m += pow(static_cast<double>(x), p) * pow(static_cast<double>(y), q) * I[x][y];
      }
    return m;
  }

  double computePhotometricMoment(const vpImage<unsigned char> &I, const vpImage<unsigned char> &M,
          int p, int q) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    double m = 0.0;
    for (unsigned int x = 0; x < height; ++x)
      for (unsigned int y = 0; y < width; ++y) {
  if (M(x,y))
    m += pow(static_cast<double>(x), p) * pow(static_cast<double>(y), q) * I[x][y];
      }
    return m;
  }

  void computeBarycenter(const vpImage<unsigned char> &I, double &xc, double &yc) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    double I_sum = sum(I);
    xc = 0.0;
    yc = 0.0;
    for (unsigned int x = 0; x < height; ++x)
      for (unsigned int y = 0; y < width; ++y) {
  xc += x * I[x][y];
  yc += y * I[x][y];
      }
    xc /= I_sum;
    yc /= I_sum;
  }

  void computeBarycenter(const vpImage<unsigned char> &I, const vpImage<unsigned char> &M,
       double &xc, double &yc) {
    unsigned int height(I.getHeight()), width(I.getWidth());
    double I_sum = 0.0;
    xc = 0.0;
    yc = 0.0;
    for (unsigned int x = 0; x < height; ++x)
      for (unsigned int y = 0; y < width; ++y) {
  if (M(x,y)) {
    xc += x * I[x][y];
    yc += y * I[x][y];
    I_sum += I[x][y];
  }
      }
    xc /= I_sum;
    yc /= I_sum;
  }

  double interpolate(const vpImage<unsigned char> &I, double x, double y, InterpolationType it) {
    switch(it) {
    case NEAREST:
      return I(vpMath::round(x), vpMath::round(y));
    case BILINEAR: {
      int x1 = (int)floor(x);
      int x2 = (int)ceil(x);
      int y1 = (int)floor(y);
      int y2 = (int)ceil(y);
      double v1, v2;
      if (x1 == x2) {
  v1 = I(x1, y1);
  v2 = I(x1, y2);
      } else {
  v1 = (x2 - x) * I(x1, y1) + (x - x1) * I(x2, y1);
  v2 = (x2 - x) * I(x1, y2) + (x - x1) * I(x2, y2);
      }
      if (y1 == y2)
  return v1;
      return (y2 - y) * v1 + (y - y1) * v2;
    }
    case BICUBIC:
      std::cerr << "Error: bi-cubic interpolation is not implemented." << std::endl;
      exit(EXIT_FAILURE);
    default:
      std::cerr << "Error: invalid interpolation type ("
    << it << ")" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  void resample(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, InterpolationType it)
  {
    unsigned int x_s = Src.getHeight();
    unsigned int y_s = Src.getWidth();
    unsigned int x_d = Dst.getHeight();
    unsigned int y_d = Dst.getWidth();
    double sx = static_cast<double>(x_s) / static_cast<double>(x_d);
    double sy = static_cast<double>(y_s) / static_cast<double>(y_d);
    for (unsigned int x = 0; x < x_d; ++x) {
      for (unsigned int y = 0; y < y_d; ++y) {
        Dst(x, y, (unsigned char)interpolate(Src, x * sx, y * sy, it));
      }
    }
  }

  void extract(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, const usRectangle &r) {
    unsigned int x_d = vpMath::round(r.getHeight());
    unsigned int y_d = vpMath::round(r.getWidth());
    double x1 = r.getX1();
    double y1 = r.getY1();
    double t = r.getOrientation();
    Dst.resize(x_d, y_d);
    for (unsigned int x = 0; x < x_d; ++x)
      for (unsigned int y = 0; y < y_d; ++y)
  Dst(x, y, (unsigned char)interpolate(Src, x1 + x * cos(t) + y * sin(t), y1 - x * sin(t) + y * cos(t),
            usImageMathematics::BILINEAR));
  }

  void extract(const vpImage<unsigned char> &Src, vpImage<double> &Dst, const usRectangle &r) {
    unsigned int x_d = vpMath::round(r.getHeight());
    unsigned int y_d = vpMath::round(r.getWidth());
    double x1 = r.getX1();
    double y1 = r.getY1();
    double t = r.getOrientation();
    Dst.resize(x_d, y_d);
    for (unsigned int x = 0; x < x_d; ++x)
      for (unsigned int y = 0; y < y_d; ++y)
  Dst(x, y, interpolate(Src, x1 + x * cos(t) + y * sin(t), y1 - x * sin(t) + y * cos(t),
            usImageMathematics::BILINEAR));
  }

}
