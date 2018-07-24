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

#include <visp3/ustk_elastography/usMotionEstimation.h>

#if defined(USTK_HAVE_ARMADILLO) && (ARMA_VERSION_MAJOR > 6) && (ARMA_VERSION_MAJOR > 700)
usMotionEstimation::usMotionEstimation() {}

usMotionEstimation::~usMotionEstimation() {}

/*!
 * \brief init
 * \param M1: RF frame 1 type mat
 * \param M2: RF frame 2 type mat
 * \param blk_w: block width (int)
 * \param blk_h: block height (int)
 * \param sr_w: search region width (int)
 * \param sr_h: search region height (int)
 */
void usMotionEstimation::init(mat M1, mat M2, int blk_w, int blk_h, int sr_w, int sr_h)
{
  m_M1 = M1;
  m_M2 = M2;
  m_Wblock = blk_w;
  m_Hblock = blk_h;
  m_WSregion = sr_w;
  m_HSregion = sr_h;

  m_Lx = floor((float)blk_w / 2.0);
  m_Ly = floor((float)blk_h / 2.0);
  m_blk_rngX = regspace<ivec>(-m_Lx, m_Lx - 1);
  m_blk_rngY = regspace<ivec>(-m_Ly, m_Ly - 1);
  m_xcRng = regspace<ivec>(sr_w + m_Lx, blk_w, m_M1.n_cols - (sr_w + m_Lx) - 1);
  m_ycRng = regspace<ivec>(sr_h + m_Ly, blk_h, m_M1.n_rows - (sr_h + m_Ly) - 1);

  m_U = mat(m_ycRng.n_elem, m_xcRng.n_elem);
  m_V = mat(m_ycRng.n_elem, m_xcRng.n_elem);
}
/*!
 * \brief init
 * \param usM1: RF frame 1 type usImageRF2D<short int>
 * \param usM2: RF frame 2 type usImageRF2D<short int>
 * \param blk_w: block width (int)
 * \param blk_h: block height (int)
 * \param sr_w: search region width (int)
 * \param sr_h: search region height (int)
 */
void usMotionEstimation::init(const usImageRF2D<short int> &usM1, const usImageRF2D<short int> &usM2, int blk_w,
                              int blk_h, int sr_w, int sr_h)
{
  m_M1 = convert_usImageRF2mat(usM1);
  m_M2 = convert_usImageRF2mat(usM2);
  /*m_M1.save("precomp.dat", raw_ascii);
  m_M2.save("postcomp.dat", raw_ascii);*/
  m_Wblock = blk_w;
  m_Hblock = blk_h;
  m_WSregion = sr_w;
  m_HSregion = sr_h;

  m_Lx = floor((float)blk_w / 2.0);
  m_Ly = floor((float)blk_h / 2.0);
  m_blk_rngX = regspace<ivec>(-m_Lx, m_Lx - 1);
  m_blk_rngY = regspace<ivec>(-m_Ly, m_Ly - 1);
  m_xcRng = regspace<ivec>(sr_w + m_Lx, blk_w, m_M1.n_cols - (sr_w + m_Lx) - 1);
  m_ycRng = regspace<ivec>(sr_h + m_Ly, blk_h, m_M1.n_rows - (sr_h + m_Ly) - 1);

  m_U = mat(m_ycRng.n_elem, m_xcRng.n_elem);
  m_V = mat(m_ycRng.n_elem, m_xcRng.n_elem);
}
/*!
 * \brief usMotionEstimation::run
 * This function run the BMA algorithm and stores the displacements*/
/* REMOVE DOXYGEN WARNING DUE TO \mathbf
 * between \f$ \mathbf{M}_1\f$ and \f$ \mathbf{M}_2\f$ in \mathbf{U} (lateral)
* and \mathbf{V} (axial).
*/
void usMotionEstimation::run()
{
  // std::cout << "Limits: " << m_ycRng.n_elem << ", " << m_xcRng.n_elem << std::endl;
  for (unsigned int i = 0; i < m_ycRng.n_elem; i++) {
    for (unsigned int j = 0; j < m_xcRng.n_elem; j++) {
      int xc = m_xcRng(j);
      int yc = m_ycRng(i);
      mat Block = m_M1(span(yc + m_blk_rngY(0), yc + m_blk_rngY(m_blk_rngY.n_elem - 1)),
                       span(xc + m_blk_rngX(0), xc + m_blk_rngX(m_blk_rngX.n_elem - 1)));
      vec t_m1 = FullSearch(Block, xc, yc, m_WSregion, m_HSregion);
      m_U(i, j) = t_m1(0);
      m_V(i, j) = t_m1(1);
    }
  }
  m_U = MedianFilt2D(m_U, 5, 7); // MedianFilt2D(m_U, 9,7);
  m_V = MedianFilt2D(m_V, 5, 7); // MedianFilt2D(m_V, 9,7);
}

vec usMotionEstimation::FullSearch(mat B1, int xc, int yc, int sr_w, int sr_h)
{
  double SAD;
  double SAD_min = 1e9;
  double x_min = 0, y_min = 0;
  vec Mi(2), Mo(2);
  // for(int i = -sr_h; i < sr_h; i++)
  for (int i = sr_h; i >= 0; i--) {
    for (int j = -sr_w; j < sr_w; j++) {
      int xt = xc + j;
      int yt = yc + i;
      mat B2 = m_M2(span(yt + m_blk_rngY(0), yt + m_blk_rngY(m_blk_rngY.n_elem - 1)),
                    span(xt + m_blk_rngX(0), xt + m_blk_rngX(m_blk_rngX.n_elem - 1)));
      SAD = accu(abs(B1 - B2)) / (double)(B1.n_rows * B1.n_cols);
      if (SAD < SAD_min) {
        SAD_min = SAD;
        x_min = xt;
        y_min = yt;
      }
      Mi(0) = xc - x_min;
      Mi(1) = yc - y_min;
    }
  }
  // Taylor Refinement
  mat B_ref = m_M2(span(y_min + m_blk_rngY(0), y_min + m_blk_rngY(m_blk_rngY.n_elem - 1)),
                   span(x_min + m_blk_rngX(0), x_min + m_blk_rngX(m_blk_rngX.n_elem - 1)));

  // Motion Vector (fractional part)
  vec Taylor_sol = TaylorApp(B1, B_ref);

  // Motion Vector (overall)
  Mo = Mi + Taylor_sol;
  return Mo;
}

vec usMotionEstimation::TaylorApp(mat B1, mat B2)
{
  mat dfx = xDifferential(B1);
  mat dfy = yDifferential(B1);

  double a = accu(dfx % dfx);
  double b = accu(dfx % dfy);
  double d = accu(dfy % dfy);

  mat z = B2 - B1;
  double p = accu(z % dfx);
  double q = accu(z % dfy);

  mat A(2, 2);
  A << a << b << endr << b << d << endr;

  mat rhs(2, 1);
  rhs << p << endr << q << endr;

  vec x = vectorise(solve(A, rhs));
  return x;
}
/*!
 * \brief usMotionEstimation::xDifferential
 * \param input matrix \f$ M(x,y) \f$
 * \return matrix with \f$ \frac{\partial M}{\partial x} \f$
 */
mat usMotionEstimation::xDifferential(mat input)
{
  int nrows, ncols;
  nrows = input.n_rows;
  ncols = input.n_cols;
  mat out(nrows, ncols);
  // Edges
  out.col(0) = diff(input(span(0, nrows - 1), span(0, 1)), 1, 1);
  out.col(ncols - 1) = diff(input(span(0, nrows - 1), span(ncols - 2, ncols - 1)), 1, 1);
  // inner Matrix
  for (int i = 1; i < (ncols - 1); i++)
    out.col(i) = (input.col(i + 1) - input.col(i - 1)) * 0.5;
  return out;
}
/*!
 * \brief usMotionEstimation::yDifferential
 * \param input matrix \f$ M(x,y) \f$
 * \return matrix with \f$ \frac{\partial M}{\partial y} \f$
 */
mat usMotionEstimation::yDifferential(mat input)
{
  int nrows, ncols;
  nrows = input.n_rows;
  ncols = input.n_cols;
  mat out(nrows, ncols);
  // Edges
  out.row(0) = diff(input(span(0, 1), span(0, ncols - 1)), 1, 0);
  out.row(nrows - 1) = diff(input(span(nrows - 2, nrows - 1), span(0, ncols - 1)), 1, 0);
  // inner Matrix
  for (int i = 1; i < (nrows - 1); i++)
    out.row(i) = (input.row(i + 1) - input.row(i - 1)) * 0.5;
  return out;
}

inline bool inside(int row, int col, int height, int width)
{
  return ((row >= 0) && (row < height) && ((col >= 0) && (col < width)));
}

inline double getpixel(mat M, int row, int col)
{
  while (!inside(row, col, M.n_rows, M.n_cols)) {
    if (row < 0)
      row++;
    else if (row >= (int)M.n_rows)
      row--;
    if (col < 0)
      col++;
    else if (col >= (int)M.n_cols)
      col--;
  }
  return M(row, col);
}

mat usMotionEstimation::MedianFilt2D(mat M, int kw, int kh)
{
  mat out(M.n_rows, M.n_cols);
  int middle = (kw * kh) / 2 + 1;
  std::map<double, int> tree;
  for (int row = 0; row < (int)M.n_rows; row++) {
    for (int col = 0; col < (int)M.n_cols; col++) {
      for (int r = -kh / 2; r < kh / 2; r++) {
        for (int c = -kw / 2; c < kw / 2; c++) {
          double av = getpixel(M, row + r, col + c);
          std::map<double, int>::iterator it = tree.find(av);
          if (it != tree.end())
            it->second++;
          else
            tree.insert(std::pair<double, int>(av, 1));
        }
      }
      int sum = 0;
      std::map<double, int>::const_iterator it;
      for (it = tree.begin(); it != tree.end(); it++) {
        sum += it->second;
        if (sum >= middle)
          break;
      }
      out(row, col) = it->first;
      tree.clear();
    }
  }
  return out;
}

void usMotionEstimation::saveU(const char *t_s) { m_U.save(t_s, raw_ascii); }

void usMotionEstimation::saveV(const char *t_s) { m_V.save(t_s, raw_ascii); }

mat usMotionEstimation::convert_usImageRF2mat(const usImageRF2D<short int> &vI)
{
  mat I(vI.getHeight(), vI.getWidth());
  for (uint i = 0; i < vI.getHeight(); i++)
    for (uint j = 0; j < vI.getWidth(); j++)
      (*(I.memptr() + i * vI.getWidth() + j)) = (double)vI(i, j);
  return I;
}

vpMatrix usMotionEstimation::convert_mat2vpMatrix(mat vI)
{
  vpMatrix I(vI.n_rows, vI.n_cols);
  memcpy(I.data, vI.memptr(), vI.size() * sizeof(double));
  return I;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_ustk_elastograpy.a(usMotionEstimation.cpp.o) has no symbols
void dummy_vpusMotionEstimation(){};
#endif // ARMADILLO
