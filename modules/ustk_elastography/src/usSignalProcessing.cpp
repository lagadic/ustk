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
 * Pedro Patlan Rosales
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_elastography/usSignalProcessing.h>

/*!
 * \brief usSignalProcessing::usSignalProcessing
 * Constructor
 */
usSignalProcessing::usSignalProcessing() {}
/*!
 * \brief usSignalProcessing::~usSignalProcessing
 * Destructor - void
 */
usSignalProcessing::~usSignalProcessing() {}

/**
* Gaussian filter kernel generator.
*
* @param height Gaussian kernel height.
* @param width Gaussian kernel width.
* @param sigma Standard deviation of the gaussian function.
*
* @return The gaussian filter kernel.
*/
vpMatrix usSignalProcessing::GaussianFilter(int height, int width, double sigma)
{
  int i, j, a, b;
  double mul;
  vpMatrix t_out(height, width);
  a = (width - 1) / 2;
  b = (height - 1) / 2;
  for (i = -a; i <= a; i++) {
    for (j = -b; j <= b; j++) {
      mul = (exp((-1.0 * ((i * i) + (j * j))) / (2.0 * sigma * sigma))) / (2 * M_PI * sigma);
      t_out[j + b][i + a] = mul;
    }
  }
  t_out = t_out / t_out.getMaxValue();
  return t_out;
}

/**
* Computes a gradient filter on a RF image, along every row.
* The filter kernel is : [-0.5, 0, 0.5]
*
* @param image RF image to apply the X gradient filter on.
* @return The output of the X gradient filter.
*/
vpMatrix usSignalProcessing::getXGradient(const usImageRF2D<short int> &image)
{
  vpMatrix t_out(image.getHeight(), image.getWidth());

  // manage first and last of each column
  for (uint i = 0; i < image.getHeight(); i++) {
    t_out[i][0] = image(i, 1) - image(i, 0);
    t_out[i][image.getWidth() - 1] = image(i, image.getWidth() - 2) - image(i, image.getWidth() - 1);
  }

  for (uint i = 0; i < image.getHeight(); i++)
    for (uint j = 1; j < image.getWidth() - 1; j++)
      t_out[i][j] = (image(i, j + 1) - image(i, j - 1)) * 0.5;

  return t_out;
}

/**
* Computes a gradient filter on a RF image, along every row.
* The filter kernel is the transpose of : [-0.5, 0, 0.5]
*
* @param image RF image to apply the Y gradient filter on.
* @return The output of the Y gradient filter.
*/
vpMatrix usSignalProcessing::getYGradient(const usImageRF2D<short> &image)
{
  vpMatrix t_out(image.getHeight(), image.getWidth());

  // manage first and last of each row
  for (uint j = 0; j < image.getWidth(); j++) {
    t_out[0][j] = image(1, j) - image(0, j);
    t_out[image.getHeight() - 1][j] = image(image.getHeight() - 1, j) - image(image.getHeight() - 2, j);
  }

  for (uint i = 1; i < image.getHeight() - 1; i++)
    for (uint j = 0; j < image.getWidth(); j++)
      t_out[i][j] = (image(i + 1, j) - image(i - 1, j)) * 0.5;

  return t_out;
}

/**
* Computes the difference between 2 matrix (value by value)
* The output matrix O is defined as O[i][j] = A[i][j] - B[i][j] for every component of the matrix.
* A and B must have the same size.
*
* @param A First input matrix.
* @param B Second input matrix.
* @return The difference matrix.
*/
vpMatrix usSignalProcessing::Difference(const usImageRF2D<short int> &A, const usImageRF2D<short int> &B)
{
  if (A.getHeight() != B.getHeight() || A.getWidth() != B.getWidth())
    throw(vpException(vpException::dimensionError,
                      "usSignalProcessing::Difference(), input matrices dimentions mismatch."));

  vpMatrix t_out(A.getHeight(), A.getWidth());

  for (uint i = 0; i < A.getHeight(); i++)
    for (uint j = 0; j < A.getWidth(); j++)
      t_out[i][j] = A(i, j) - B(i, j);
  return t_out;
}

/**
* Performs a bilinear interpolation on matrix In, to produce an output matrix of size (newH,newW)
*
* @param In Input matrix to interpolate.
* @param newW Output matrix width.
* @param newH Output matrix height.
* @return The resulting matrix of the interpolation.
*/
vpMatrix usSignalProcessing::BilinearInterpolation(vpMatrix In, uint newW, uint newH)
{
  vpMatrix ivec(newH, newW);
  double tx = (double)(In.getCols() - 1.0) / newW;
  double ty = (double)(In.getRows() - 1.0) / newH;
  double x_diff, y_diff;
  double A, B, C, D;

  for (uint i = 0; i < newH; i++) {
    for (uint j = 0; j < newW; j++) {
      int x = (int)(tx * j);
      int y = (int)(ty * i);
      x_diff = ((tx * j) - x);
      y_diff = ((ty * i) - y);

      A = In[y][x];
      B = In[y][x + 1];
      C = In[y + 1][x];
      D = In[y + 1][x + 1];

      double vi = (double)(A * (1.0 - x_diff) * (1.0 - y_diff) + B * (x_diff) * (1.0 - y_diff) +
                           C * (y_diff) * (1.0 - x_diff) + D * (x_diff * y_diff));
      ivec.data[j + i * newW] = vi;
    }
  }
  return ivec;
}

/**
* Performs the Hadamar product between 2 input matrices.
* matrix1 and matrix2 must have the same size.
*
* @param matrix1 First input matrix to multiply.
* @param matrix2 Second input matrix to multiply.
* @return The resulting matrix of the product.
*/
vpMatrix usSignalProcessing::HadamardProd(vpMatrix matrix1, vpMatrix matrix2)
{
  assert(matrix1.getRows() == matrix2.getRows());
  assert(matrix1.getCols() == matrix2.getCols());
  uint t_rows = matrix2.getRows();
  uint t_cols = matrix2.getCols();
  vpMatrix t_out(t_rows, t_cols);
  for (uint i = 0; i < t_rows; i++)
    for (uint j = 0; j < t_cols; j++)
      t_out[i][j] = matrix1[i][j] * matrix2[i][j];
  return t_out;
}
