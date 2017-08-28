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

#if defined(USTK_HAVE_FFTW)

/**
* Constructor.
* @param decimationFactor Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
usRFToPreScan2DConverter::usRFToPreScan2DConverter(int decimationFactor) : m_logCompressor(),
  m_decimationFactor(decimationFactor), m_isInit(false) {

}

/**
* Destructor.
*/
usRFToPreScan2DConverter::~usRFToPreScan2DConverter() {
  if (m_isInit) {
    fftw_free(m_fft_in); fftw_free(m_fft_out); fftw_free(m_fft_conv); fftw_free(m_fft_out_inv);
    fftw_destroy_plan(m_p); fftw_destroy_plan(m_pinv);
	delete m_env;
	delete m_comp;
  }
}


/**
* Init method, to pre-allocate memory for all the processes (fft inputs/outputs, log compression output).
* @param widthRF Width of the RF frames to convert : number of scanlines.
* @param heigthRF Height of the RF frames to convert : number of RF samples.
*/
void usRFToPreScan2DConverter::init(int widthRF, int heigthRF) {
  if (m_isInit && (m_signalSize != heigthRF || m_scanLineNumber != widthRF)) {
    fftw_free(m_fft_in); fftw_free(m_fft_out); fftw_free(m_fft_conv); fftw_free(m_fft_out_inv);
    fftw_destroy_plan(m_p); fftw_destroy_plan(m_pinv);
	delete m_env;
	delete m_comp;
  }
  else if(m_signalSize == heigthRF && m_scanLineNumber == heigthRF)
    return;

  // for FFT
  m_fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * heigthRF);
  m_fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * heigthRF);
  m_fft_conv = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * heigthRF);
  m_fft_out_inv = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * heigthRF);

  // log compression
  m_env = new double[heigthRF * widthRF];
  m_comp = new unsigned char[heigthRF * widthRF];

  m_signalSize = heigthRF;
  m_scanLineNumber = widthRF;

  m_p = fftw_plan_dft_1d(m_signalSize, m_fft_in, m_fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
  m_pinv = fftw_plan_dft_1d(m_signalSize, m_fft_out, m_fft_out_inv, FFTW_BACKWARD, FFTW_ESTIMATE);

  m_isInit = true;
}

/*!
 * \brief SProcessing::m_Hilbert
 * This function computes the Hilbert transform of a signal s
 * \param s: Signal of 16-bit values
 * \return std::vector<std::complex<double> > that contains the Hilbert transform
 */
void usRFToPreScan2DConverter::enveloppeDetection(const short int *s, double* out)
{
  ///time of Hilbert transform
  //const clock_t begin_time = clock();
  int N = m_signalSize;

  // Put signal data s into in
  for(int i = 0; i < N; i++)
  {
    m_fft_in[i][0] = (double)s[i];
    m_fft_in[i][1] = 0.0;
  }

  // Obtain the FFT
  fftw_execute(m_p);

  for(int i= 0; i < N; i++)
  {

    if(i<N/2)
      m_fft_out[i][1]=-m_fft_out[i][1];
    else if(i==N/2)
    {
      m_fft_out[i][0]=0;
      m_fft_out[i][1]=0;
    }
    else if(i>N/2)
      m_fft_out[i][0]=-m_fft_out[i][0];
    if(i==0)
    {
      m_fft_out[i][0]=0;
      m_fft_out[i][1]=0;
    }

    double z=m_fft_out[i][1];
    m_fft_out[i][1]=m_fft_out[i][0];
    m_fft_out[i][0]=z;
  }

  // FFT Inverse

  // Obtain the IFFT
  fftw_execute(m_pinv);
  // Put the iFFT output in sa
  double Ndouble = (double)N;
  for(int i = 0; i < N; i++)
  {
    out[i] = (unsigned char) sqrt(abs(std::complex<double> (m_fft_in[i][0], -m_fft_out_inv[i][0]/ Ndouble)));
  }
}

/**
* Convert method : performs the conversion from RF frame to a pre-scan frame using the following processes :
* - Enveloppe detector
* - Logarithmic compression
* - Decimation
*
* @param rfImage RF frame to convert
* @param preScanImage pre-scan image : result of convertion
*/
void usRFToPreScan2DConverter::convert(const usImageRF2D<short int> &rfImage, usImagePreScan2D<unsigned char> &preScanImage) {

  if(!m_isInit || ((int)rfImage.getWidth()) != m_scanLineNumber || ((int)rfImage.getHeight()) != m_signalSize) {
    init(rfImage.getWidth(), rfImage.getHeight());
  }
  preScanImage.resize(rfImage.getHeight() / m_decimationFactor,rfImage.getWidth());

  // First we copy the transducer settings
  preScanImage.setImagePreScanSettings(rfImage);

  int w = rfImage.getWidth();
  int h = rfImage.getHeight();

  unsigned int frameSize = w*h;

  // Run envelope detector
  for (int i = 0; i < w; ++i) {
    enveloppeDetection(rfImage.bitmap + i*h, m_env + i * h);
  }

  // Log-compress
  m_logCompressor.run(m_comp, m_env, frameSize);

  //find min & max values
  double min = 1e8;
  double max = -1e8;
  for (unsigned int i = 0; i < frameSize; ++i) {
    if (m_comp[i] < min)
      min = m_comp[i];
    if (m_comp[i] > max)
      max = m_comp[i];
  }

  //max-min computation
  double maxMinDiff = max - min;

  //Decimate and normalize
  int k = 0;
  for (int i = 0; i < h; i+=m_decimationFactor) {
    for (int j = 0; j < w ; ++j) {
      unsigned int  vcol = (unsigned int) (((m_comp[i + h * j] - min) / maxMinDiff) * 255);
      preScanImage[k][j] = (vcol>255)?255:vcol;
    }
    k++;
  }
}

/**
* Decimation factor getter.
* @return Decimation factor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
int usRFToPreScan2DConverter::getDecimationFactor() {
  return m_decimationFactor;
}

/**
* Decimation factor setter.
* @param  decimationFactor : keep only 1 pre-scan sample every N sample (N = decimationFactor)
*/
void usRFToPreScan2DConverter::setDecimationFactor(int decimationFactor) {
  m_decimationFactor = decimationFactor;
}

#endif
