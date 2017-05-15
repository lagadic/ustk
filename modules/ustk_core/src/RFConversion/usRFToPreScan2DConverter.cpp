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
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPreScan2DConverter.cpp
 * @brief 2D scan-converter
 */

#include <visp3/ustk_core/usRFToPreScan2DConverter.h>

usRFToPreScan2DConverter::usRFToPreScan2DConverter() {

}

usRFToPreScan2DConverter::~usRFToPreScan2DConverter() {

}

/*!
 * \brief SProcessing::m_Hilbert
 * This function computes the Hilbert transform of a signal s
 * \param s: Signal of 16-bit values
 * \return std::vector<std::complex<double> > that contains the Hilbert transform
 */
std::vector<std::complex<double> > usRFToPreScan2DConverter::HilbertTransform(const short int *s, int size)
{
  ///time of Hilbert transform
  //const clock_t begin_time = clock();
  std::vector<std::complex<double> > a, b, sa;
  fftw_complex *in, *out, *conv, *out_inv;
  fftw_plan p, pinv;
  int N = size;

  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  conv = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  // Put signal data s into in
  for(int i = 0; i < N; i++)
  {
    in[i][0] = (double)s[i];
    in[i][1] = 0.0;
  }

  // Obtain the FFT
  fftw_execute(p);

  double z;
  for(int i= 0; i < N; i++)
  {

    if(i<N/2)
      out[i][1]=-1*out[i][1];
    if(i==N/2)
    {
      out[i][0]=0;
      out[i][1]=0;
    }
    if(i>N/2)
      out[i][0]=-1*out[i][0];
    if(i==0)
    {
      out[i][0]=0;
      out[i][1]=0;
    }

    z=out[i][1];
    out[i][1]=out[i][0];
    out[i][0]=z;
  }

  // FFT Inverse
  out_inv = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
  pinv = fftw_plan_dft_1d(N, out, out_inv, FFTW_BACKWARD, FFTW_ESTIMATE);
  // Obtain the IFFT
  fftw_execute(pinv);
  fftw_destroy_plan(p); fftw_destroy_plan(pinv);
  // Put the iFFT output in sa
  for(int i = 0; i < N; i++)
  {
    sa.push_back(std::complex<double> (in[i][0], -out_inv[i][0]/(double)N));
    //cout << real(sa.at(i)) <<"," << imag(sa.at(i)) <<" i" <<endl ;
    //std::cin.ignore();
  }
  fftw_free(in); fftw_free(out); fftw_free(conv); fftw_free(out_inv);
  return sa;
}

std::vector<double> usRFToPreScan2DConverter::sqrtAbsv(std::vector<std::complex<double> > cv)
{
  std::vector<double> out;
  for(unsigned int i = 0; i < cv.size(); i++)
  {
    out.push_back(sqrt(abs(cv.at(i))));
  }
  return out;
}

void usRFToPreScan2DConverter::convert(const usImageRF2D<short int> &rfImage, usImagePreScan2D<unsigned char> &preScanImage) {

  std::cout << "1" << std::endl;
  preScanImage.resize(rfImage.getHeight(),rfImage.getWidth());

  std::cout << "2" << std::endl;
  // First we copy the transducer settings
  preScanImage.setImagePreScanSettings(rfImage);

  std::cout << "3" << std::endl;

  std::cout << "rd LN = " << rfImage.getScanLineNumber() << std::endl;
  for(unsigned int i = 0; i < rfImage.getScanLineNumber(); i++) {
    // computing square root of Hilbert transform norm for each scanline's signal
    std::vector<double> data = sqrtAbsv(HilbertTransform(rfImage[i],rfImage.getRFSampleNumber()));

    // now we normalize each sample between [0;254]
    double max = (double)*std::max_element(data.begin(),data.end());
    for (unsigned int j=0;j<data.size();j++) {
      double m = data[i];
      m /= max;
      m *= 255;
      m = nearbyint(m);
      preScanImage[j][i] = (unsigned char)m;
    }

    std::cout << "scanline " << i << "converted" << std::endl;
  }

    std::cout << "end convert" << std::endl;
}
