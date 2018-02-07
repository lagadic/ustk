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

#include <visp3/ustk_elastography/usConvolution2d.h>

#if defined(USTK_HAVE_FFTW)

/**
* Default constructor.
* It only initializes the pointers class members to NULL.
*/
usConvolution2d::usConvolution2d()
  : outa(NULL), outb(NULL), outc(NULL), out(NULL), ad(NULL), bd(NULL), p1(), p2(), p3(), m_init(false)
{
}

/**
* Destructor.
* Clear memory allocations.
*/
usConvolution2d::~usConvolution2d()
{
  fftw_destroy_plan(p1);
  fftw_destroy_plan(p2);
  fftw_destroy_plan(p3);
  delete outa;
  delete outb;
  delete outc;
  delete out;
  delete ad;
  delete bd;
}

/**
* Initialization of the convolution process with the matrix dimentions specified (if not already done), and storage of
* convolution inputs.
*
* @param matrix1 Input matrix for the convolution.
* @param matrix2 Convolution filter to apply on matrix1.
*/
void usConvolution2d::init(const vpMatrix &matrix1, const vpMatrix &matrix2)
{
  // check matrix dimentions
  if (Am != matrix1.getRows() || An != matrix1.getCols() || Bm != matrix2.getRows() || Bn != matrix2.getCols()) {

    // memory de-allocation to avoid leak
    delete outa;
    delete outb;
    delete outc;
    delete out;
    delete ad;
    delete bd;

    fftw_destroy_plan(p1);
    fftw_destroy_plan(p2);
    fftw_destroy_plan(p3);

    // re-allocation with new matrices dimentions
    Am = matrix1.getRows();
    An = matrix1.getCols();
    Bm = matrix2.getRows();
    Bn = matrix2.getCols();

    h_dst = Am - Bm + 1; // Valid convolution
    w_dst = An - Bn + 1;
    hf = Am + Bm - 1; // Full convolution
    wf = An + Bn - 1;

    m_R.resize(h_dst, w_dst);
    m_M1 = matrix1;
    m_M2 = matrix2;

    // fftw inputs/outputs arrays
    ad = new fftw_complex[hf * wf];
    bd = new fftw_complex[hf * wf];
    outa = new fftw_complex[hf * wf];
    outb = new fftw_complex[hf * wf];
    outc = new fftw_complex[hf * wf];
    out = new fftw_complex[hf * wf];

    // fftw plans
    p1 = fftw_plan_dft_2d(wf, hf, ad, outa, FFTW_FORWARD, FFTW_ESTIMATE);
    p2 = fftw_plan_dft_2d(wf, hf, bd, outb, FFTW_FORWARD, FFTW_ESTIMATE);
    p3 = fftw_plan_dft_2d(wf, hf, outc, out, FFTW_BACKWARD, FFTW_ESTIMATE);
  } else { // init already done with correct matrix dimentions
    m_M1 = matrix1;
    m_M2 = matrix2;
  }

  padding_zeros();
  m_init = true;
}

/**
* Run the convolution.
*
* @param matrix1 Input matrix for the convolution.
* @param matrix2 Convolution filter to apply on matrix1.
*/
vpMatrix usConvolution2d::run(const vpMatrix &matrix1, const vpMatrix &matrix2)
{
  init(matrix1, matrix2);

  fftw_execute(p1);
  fftw_execute(p2);
  // Complex product
  for (uint i = 0; i < wf * hf; ++i) {
    outc[i][0] = (outa[i][0] * outb[i][0]) - (outa[i][1] * outb[i][1]);
    outc[i][1] = (outa[i][0] * outb[i][1]) + (outa[i][1] * outb[i][0]);
  }
  fftw_execute(p3);
  // Storing only the valid convolution
  int startX = vpMath::round((double)(wf - w_dst) / 2.0);
  int startY = vpMath::round((double)(hf - h_dst) / 2.0);
  int endX = startX + w_dst;
  int endY = startY + h_dst;
  int k = 0;
  int l;
  for (int i = startX; i < endX; ++i) {
    l = 0;
    for (int j = startY; j < endY; ++j) {
      // the output of the convolution array (out) is extracted column by column.
      m_R[l][k] = out[j + i * hf][0] / (double)(wf * hf);
      l++;
    }
    k++;
  }

  return m_R;
}

void usConvolution2d::padding_zeros()
{
  // Padding the two arrays with zeros
  for (uint i = 0; i < wf; ++i) {
    for (uint j = 0; j < hf; ++j) {
      // fftw intput arrays (ad, bd) are filled scanline per scanline (column by column)
      ad[j + i * hf][0] = ((j < Am) && (i < An)) ? m_M1[j][i] : 0.0;
      ad[j + i * hf][1] = 0.0;
      bd[j + i * hf][0] = ((j < Bm) && (i < Bn)) ? m_M2[j][i] : 0.0;
      bd[j + i * hf][1] = 0.0;
    }
  }
}
#endif // USTK_HAVE_FFTW
