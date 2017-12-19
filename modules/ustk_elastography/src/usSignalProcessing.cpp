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

vpMatrix usSignalProcessing::m_FIR1(unsigned N, double Wn)
{
  unsigned odd, i, j, nhlf;
  double f1, gain, c1;
  double wind[N + 1], xn[(N + 1) / 2], b[(N + 1) / 2], c[(N + 1) / 2], c3[(N + 1) / 2];
  vpMatrix bb(N + 1, 1);
  // Initial parameters
  gain = 0.0000000;
  N = N + 1;
  odd = N - (N / 2) * 2;
  ///////// Hamming(N)  ////////
  for (i = 0; i < (N + 1); i++)
    wind[i] = 0.54 - 0.46 * cos((2 * PI * i) / (N - 1));
  f1 = Wn / 2.0;
  c1 = f1;
  nhlf = (N + 1) / 2; // Half of elements
  /// Lowpass  ///
  if (odd)
    b[0] = 2 * c1;
  for (i = 0; i < nhlf; i++)
    xn[i] = i + 0.5 * (1 - odd);
  for (i = 0; i < nhlf; i++)
    c[i] = PI * xn[i];
  for (i = 0; i < nhlf; i++)
    c3[i] = 2 * c1 * c[i];
  for (i = 0; i < nhlf; i++)
    b[i] = sin(c3[i]) / c[i];
  for (i = 0, j = nhlf - 1; i < nhlf; i++, j--)
    bb.data[i] = b[j];
  for (i = nhlf, j = 0; i < (N + 1); i++, j++)
    bb.data[i] = b[j];
  for (i = 0; i < (N + 1); i++)
    bb.data[i] = bb.data[i] * wind[i];
  for (i = 0; i < (N + 1); i++)
    gain += bb.data[i];
  for (i = 0; i < (N + 1); i++)
    bb.data[i] = bb.data[i] / gain;
  return bb;
}

vpMatrix usSignalProcessing::GaussianFilter(int ny, int nx, double sigma)
{
  int i, j, a, b;
  double mul;
  vpMatrix t_out(ny, nx);
  a = (nx - 1) / 2;
  b = (ny - 1) / 2;
  for (i = -a; i <= a; i++) {
    for (j = -b; j <= b; j++) {
      mul = (exp((-1.0 * ((i * i) + (j * j))) / (2.0 * sigma * sigma))) / (2 * M_PI * sigma);
      t_out.data[(i + a) * ny + (j + b)] = mul;
    }
  }
  t_out = t_out / t_out.getMaxValue();
  return t_out;
}

vpMatrix usSignalProcessing::GetGx(vpMatrix F)
{
  assert(F.getCols() >= 0);
  vpMatrix t_out(F.getRows(), F.getCols());
  // Computing the edges of the gradient
  for (uint i = 0; i < F.getRows(); i++) {
    *(t_out.data + i) = *(F.data + i + F.getRows()) - *(F.data + i);

    *(t_out.data + (F.getCols() - 1) * F.getRows() + i) =
        *(F.data + (F.getCols() - 2) * F.getRows() + i) - *(F.data + (F.getCols() - 1) * F.getRows() + i);
  }
  //#pragma omp parallel for collapse(2)
  for (uint i = 1; i < F.getCols() - 1; i++)
    for (uint j = 0; j < F.getRows(); j++)
      *(t_out.data + i * F.getRows() + j) =
          (double)(*(F.data + (i + 1) * F.getRows() + j) - *(F.data + (i - 1) * F.getRows() + j)) * 0.5;
  return t_out;
}

vpMatrix usSignalProcessing::GetGy(vpMatrix F)
{
  assert(F.getCols() >= 0);
  vpMatrix t_out(F.getRows(), F.getCols());
  // Computing the edges of the gradient
  for (uint i = 0; i < F.getCols(); i++) {
    *(t_out.data + i * F.getRows()) = *(F.data + i * F.getRows() + 1) - *(F.data + i * F.getRows());
    *(t_out.data + (F.getRows() - 1) + i * F.getRows()) =
        *(F.data + (F.getRows() - 1) + i * F.getRows()) - *(F.data + (F.getRows() - 2) + i * F.getRows());
  }
  //#pragma omp parallel for collapse(2)
  for (uint i = 1; i < F.getRows() - 1; i++)
    for (uint j = 0; j < F.getCols(); j++)
      *(t_out.data + j * F.getRows() + i) =
          (double)(*(F.data + j * F.getRows() + i + 1) - *(F.data + j * F.getRows() + i - 1)) * 0.5;
  return t_out;
}

vpMatrix usSignalProcessing::GetGx(usImageRF2D<short int> F)
{
  assert(F.getCols() >= 0);
  vpMatrix t_out(F.getRows(), F.getCols());
  // Computing the edges of the gradient
  for (uint i = 0; i < F.getRows(); i++) {
    *(t_out.data + i) = *(F.bitmap + i + F.getRows()) - *(F.bitmap + i);

    *(t_out.data + (F.getCols() - 1) * F.getRows() + i) =
        *(F.bitmap + (F.getCols() - 2) * F.getRows() + i) - *(F.bitmap + (F.getCols() - 1) * F.getRows() + i);
  }
  //#pragma omp parallel for collapse(2)
  for (uint i = 1; i < F.getCols() - 1; i++)
    for (uint j = 0; j < F.getRows(); j++)
      *(t_out.data + i * F.getRows() + j) =
          (double)(*(F.bitmap + (i + 1) * F.getRows() + j) - *(F.bitmap + (i - 1) * F.getRows() + j)) * 0.5;
  return t_out;
}

vpMatrix usSignalProcessing::GetGy(usImageRF2D<short int> F)
{
  assert(F.getCols() >= 0);
  vpMatrix t_out(F.getRows(), F.getCols());
  // Computing the edges of the gradient
  for (uint i = 0; i < F.getCols(); i++) {
    *(t_out.data + i * F.getRows()) = *(F.bitmap + i * F.getRows() + 1) - *(F.bitmap + i * F.getRows());
    *(t_out.data + (F.getRows() - 1) + i * F.getRows()) =
        *(F.bitmap + (F.getRows() - 1) + i * F.getRows()) - *(F.bitmap + (F.getRows() - 2) + i * F.getRows());
  }
  //#pragma omp parallel for collapse(2)
  for (uint i = 1; i < F.getRows() - 1; i++)
    for (uint j = 0; j < F.getCols(); j++)
      *(t_out.data + j * F.getRows() + i) =
          (double)(*(F.bitmap + j * F.getRows() + i + 1) - *(F.bitmap + j * F.getRows() + i - 1)) * 0.5;
  return t_out;
}

vpMatrix usSignalProcessing::Difference(usImageRF2D<short int> A, usImageRF2D<short int> B)
{
  assert((A.getCols() == B.getCols()) && (A.getCols() == B.getCols()));
  vpMatrix t_out(A.getRows(), A.getCols());
  for (uint i = 0; i < A.getSize(); i++)
    *(t_out.data + i) = *(A.bitmap + i) - *(B.bitmap + i);
  return t_out;
}

vpMatrix usSignalProcessing::BilinearInterpolation(vpMatrix In, uint newW, uint newH)
{
  vpMatrix ivec(newH, newW);
  double tx = (double)(In.getCols() - 1.0) / newW;
  double ty = (double)(In.getRows() - 1.0) / newH;
  double x_diff, y_diff;
  double A, B, C, D;
  //#pragma omp parallel for collapse(2)
  for (uint i = 0; i < newH; i++) {
    for (uint j = 0; j < newW; j++) {
      int x = (int)(tx * j);
      int y = (int)(ty * i);
      x_diff = ((tx * j) - x);
      y_diff = ((ty * i) - y);

      A = In.data[x * In.getRows() + y];
      B = In.data[(x + 1) * In.getRows() + y];
      C = In.data[x * In.getRows() + (y + 1)];
      D = In.data[(x + 1) * In.getRows() + (y + 1)];

      double vi = (double)(A * (1.0 - x_diff) * (1.0 - y_diff) + B * (x_diff) * (1.0 - y_diff) +
                           C * (y_diff) * (1.0 - x_diff) + D * (x_diff * y_diff));
      ivec.data[j * newH + i] = vi;
    }
  }
  return ivec;
}

vpMatrix usSignalProcessing::HadamardProd(vpMatrix in_array1, vpMatrix in_array2)
{
  assert(in_array1.getRows() == in_array2.getRows());
  assert(in_array1.getCols() == in_array2.getCols());
  uint t_rows = in_array2.getRows();
  uint t_cols = in_array2.getCols();
  vpMatrix t_out(t_rows, t_cols);
  for (uint i = 0; i < t_cols; i++)
    for (uint j = 0; j < t_rows; j++)
      *(t_out.data + i * t_rows + j) = (*(in_array1.data + (i * t_rows) + j)) * (*(in_array2.data + (i * t_rows) + j));
  return t_out;
}

vpMatrix usSignalProcessing::Normalize_1(vpMatrix &in_array)
{
  vpMatrix out_array(in_array.getRows(), in_array.getCols());
  /*double maxv = in_array.getMaxValue();
  double minv = in_array.getMinValue();*/

  double min_str = vpMath::abs(in_array.getMaxValue());
  double max_str = vpMath::abs(in_array.getMinValue());

  double max_abs = (min_str > max_str) ? min_str : max_str;

  for (uint i = 0; i < in_array.size(); i++) {
    *(out_array.data + i) = vpMath::abs(in_array.data[i]) / max_abs;
  }
  return out_array;
}

vpMatrix usSignalProcessing::SubMatrix(const vpMatrix &M, uint r, uint c, uint nrows, uint ncols)
{
  uint rnrows = r + nrows;
  uint cncols = c + ncols;
  vpMatrix t_Mout;

  assert(rnrows < M.getRows() || cncols < M.getCols());
  t_Mout.resize(nrows, ncols);
  for (unsigned int i = c; i < cncols; i++)
    for (unsigned int j = r; j < rnrows; j++)
      t_Mout.data[(i - c) * nrows + (j - r)] = M.data[i * M.getRows() + j];

  return t_Mout;
}
