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

#include <visp3/ustk_elastography/usElastography.h>

#if defined(USTK_HAVE_FFTW)

/**
* Default constructor.
*/
usElastography::usElastography()
{
  // Using default percentage of LSQ strain
  // If you want to change this value use
  // setLSQpercentage(double per);
  m_Lsqper = 0.01; // 0.08 BMA //Just OF 0.01
  // Frame rate in fps
  m_FPS = 24.0;
  // Sampling frequency
  m_fs = 40e6;
  // speed of the sound
  m_c = 1540.0;
  m_isloadPre = false;
  m_isloadPost = false;
  m_setROI = false;
  // Using OF by default
  m_mEstimatior = OF;

  // Avoiding leaking memory with the convolutions on cpu
  for (uint i = 0; i < 6; i++)
    cC.push_back(new usConvolution2d);
}

/**
* Constructor setting the pre-compressed and the post-compressed images to perform elastography on.
*
* @param Pre Pre-compresssed RF image.
* @param Post Post-compresssed RF image.
*/
usElastography::usElastography(usImageRF2D<short int> &Pre, usImageRF2D<short int> &Post)
{
  setPreCompression(Pre);
  setPostCompression(Post);
  m_Lsqper = 0.01;
  m_FPS = 24.0;
  m_fs = 40e6;
  m_c = 1540.0;
  m_setROI = false;
  m_mEstimatior = OF;

  for (uint i = 0; i < 6; i++)
    cC.push_back(new usConvolution2d);
}

/**
* Destructor, free memory.
*/
usElastography::~usElastography()
{
  // Delete the convolution objects
  for (uint i = 0; i < 6; i++)
    delete cC.at(i);
}

/**
* Pre-compresssed RF image setter.
* @param Pre Pre-compresssed RF image.
*/
void usElastography::setPreCompression(const usImageRF2D<short int> &Pre)
{
  m_Precomp = Pre;
  m_isloadPre = true;
}

/**
* Post-compresssed RF image setter.
* @param Post Post-compresssed RF image.
*/
void usElastography::setPostCompression(const usImageRF2D<short int> &Post)
{
  m_Postcomp = Post;
  m_isloadPost = true;
}

/**
* Setter for LSQ strain percentage.
* @param per LSQ strain percentage.
*/
void usElastography::setLSQpercentage(double per) { m_Lsqper = per; }

/**
* Setter for LSQ strain percentage.
* @param fps LSQ strain percentage.
*/
void usElastography::setFPS(double fps) { m_FPS = fps; }

/**
* Setter for sampling frequency of the ultrasound wave.
* @param fs Sampling frequency in Hertz.
*/
void usElastography::setfs(double fs) { m_fs = fs; }

/**
* Updater of pre/post compressed image : sets the new image as pre-compressed image for process, and sets the old
* pre-compressed image as new post-compressed.
* @param image New pre-compressed image.
*/
void usElastography::updateRF(const usImageRF2D<short> &image)
{
  if (!m_isloadPre) {
    setPreCompression(image);
  } else {
    if (m_isloadPost)
      setPreCompression(m_Postcomp);
    setPostCompression(image);
  }
}

/**
* ROI setter: the ROI is the region in the RF image in which to compute the elastography.
* @param tx Top left column of the ROI.
* @param ty Top left row of the ROI.
* @param tw ROI width in px.
* @param tw ROI height in px.
*/
void usElastography::setROI(int tx, int ty, int tw, int th)
{
  m_ix = tx;
  m_iy = ty;
  m_rw = tw;
  m_rh = th;
  m_setROI = true;
}

/**
* ROI position update, allows to move the ROI in the image.
* @param tx New top left column of the ROI.
* @param ty New top left row of the ROI.
*/
void usElastography::updateROIPos(int tx, int ty)
{
  m_ix = tx;
  m_iy = ty;
}

usImageRF2D<short int> usElastography::usImageRF_ROI(const usImageRF2D<short int> &M, uint r, uint c, uint nrows,
                                                     uint ncols)
{
  uint rnrows = r + nrows;
  uint cncols = c + ncols;
  usImageRF2D<short int> t_Mout;
  assert(rnrows < M.getHeight() || cncols < M.getWidth());
  t_Mout.resize(nrows, ncols);
  for (unsigned int i = r; i < rnrows; i++)
    for (unsigned int j = c; j < cncols; j++)
      t_Mout(i - r, j - c, M(i, j));

  return t_Mout;
}

/**
* Run the elastography computation.
* @return The elastography image of the ROI (dark = hard tissues, white = soft).
*/
vpImage<unsigned char> usElastography::run()
{

  if (m_isloadPre == true && m_isloadPost == true && m_setROI == true) {
    m_PreROI = usImageRF_ROI(m_Precomp, m_iy, m_ix, m_rh, m_rw);
    m_PostROI = usImageRF_ROI(m_Postcomp, m_iy, m_ix, m_rh, m_rw);
    assert(m_PreROI.getWidth() == m_PostROI.getWidth());
    assert(m_PreROI.getHeight() == m_PostROI.getHeight());
    if (m_mEstimatior == BMA_TAYLOR) {
      // Step 0: BMA
      m_ME.init(m_PreROI, m_PostROI, 2, 20, 2, 120);
      m_ME.run();
      // U = m_ME.getU_vp() * (m_c * (m_PRF / (2.0 * m_fs)));
      V = m_ME.getV_vp() * (m_c * (m_FPS / (2.0 * m_fs)));
      m_h_m = m_PreROI.getHeight();
      m_w_m = m_PreROI.getWidth();
    } else {
      // Step 1: Numerical gradients
      vpMatrix Fx = usSignalProcessing::getXGradient(m_PreROI);
      vpMatrix::save("Fx", Fx);
      vpMatrix Fy = usSignalProcessing::getYGradient(m_PreROI);
      vpMatrix::save("Fy", Fy);
      // Difference
      // vpMatrix Dt = m_Postcomp-m_Precomp;
      vpMatrix Dt = usSignalProcessing::Difference(m_PostROI, m_PreROI);
      vpMatrix::save("Dt", Dt);
      // Step 2: computing the quadratic functions
      vpMatrix dx2 = usSignalProcessing::HadamardProd(Fx, Fx);
      vpMatrix::save("dx2", dx2);
      vpMatrix dy2 = usSignalProcessing::HadamardProd(Fy, Fy);
      vpMatrix::save("dy2", dy2);
      vpMatrix dxy = usSignalProcessing::HadamardProd(Fx, Fy);
      vpMatrix::save("dxy", V);
      vpMatrix dtx = usSignalProcessing::HadamardProd(Dt, Fx);
      vpMatrix::save("dtx", dtx);
      vpMatrix dty = usSignalProcessing::HadamardProd(Dt, Fy);
      vpMatrix::save("dty", dty);
      // Step 3: blur the 5 images
      vpMatrix h_gauss = usSignalProcessing::GaussianFilter(15, 5, 7.5); //(51,35,5); //21 good-phantom
      vpMatrix::save("h_gauss", h_gauss);

      vpMatrix gdx2 = cC[0]->run(dx2, h_gauss);
      vpMatrix gdy2 = cC[1]->run(dy2, h_gauss);
      vpMatrix gdxy = cC[2]->run(dxy, h_gauss);
      vpMatrix gdty = cC[3]->run(dty, h_gauss);
      vpMatrix gdtx = cC[4]->run(dtx, h_gauss);

      vpMatrix::save("gdx2", gdx2);
      vpMatrix::save("gdy2", gdy2);
      vpMatrix::save("gdxy", gdxy);
      vpMatrix::save("gdty", gdty);
      vpMatrix::save("gdtx", gdtx);

      // Step 4: compute the u, v components of the displacement
      // usign windows of size Wsizex, Wsizey
      m_h_m = gdx2.getRows();
      m_w_m = gdx2.getCols();
      int Wsizex = vpMath::round(0.1 * m_w_m); // 0.07
      int Wsizey = vpMath::round(0.1 * m_h_m); // 0.07
      int Wincx = vpMath::round(0.75 * Wsizex);
      int Wincy = vpMath::round(0.75 * Wsizey);
      int h_w = vpMath::round((double)(m_h_m - Wsizey) / (double)Wincy);
      int w_w = vpMath::round((double)(m_w_m - Wsizex) / (double)Wincx);

      assert(w_w > 1);
      assert(h_w > 1);

      // U.resize(h_w, w_w);
      V.resize(h_w, w_w);
      vpMatrix b(2, 1);
      vpMatrix M(2, 2);
      vpMatrix X(2, 1);
      uint k = 0, l;
      for (uint n = 0; n < (uint)(m_w_m - Wsizex - Wincx); n += Wincx) {
        l = 0;
        for (uint m = 0; m < (uint)(m_h_m - Wsizey - Wincy); m += Wincy) {
          double sgdx2_w = 0.0;
          double sgdy2_w = 0.0;
          double sgdxy_w = 0.0;
          double sgdtx_w = 0.0;
          double sgdty_w = 0.0;
          for (uint i = m; i < (m + Wsizey); i++) {
            for (uint j = n; j < (n + Wsizex); j++) {
              sgdx2_w += gdx2[i][j];
              sgdy2_w += gdy2[i][j];
              sgdxy_w += gdxy[i][j];
              sgdtx_w += gdtx[i][j];
              sgdty_w += gdty[i][j];
            }
          }
          M.data[0] = sgdx2_w;
          M.data[1] = sgdxy_w;
          M.data[2] = sgdxy_w;
          M.data[3] = sgdy2_w;
          b.data[0] = -sgdtx_w;
          b.data[1] = -sgdty_w;

          X = M.pseudoInverse() * b;
          // U[l][k] = X.data[0] * (m_c * (m_PRF / (2.0 * m_fs))); //lateral displacements
          V[l][k] = X.data[1] * (m_c * (m_FPS / (2.0 * m_fs))); // axial displacements
          l++;
        }
        k++;
      }
    }

    vpMatrix::save("V_mat", V);

    vpMatrix vV(m_h_m, m_w_m, true);
    vV = usSignalProcessing::BilinearInterpolation(V, m_w_m, m_h_m);
    /// Strain estimation
    /// "LSQ strain estimation"
    double kappa = (2.0 * m_fs) / (m_c * m_FPS);
    double d_m = m_Lsqper * m_h_m; // 20.0; //Good value in phantom
    double n = d_m + 1;
    // Filter kernel
    vpMatrix h((uint)n + 1, 1, true);
    double xi = kappa * 12.0 / (n * (n * n - 1));
    uint j = 0;
    for (int i = n; i >= 0; i--) {
      h[j][0] = xi * (i - (n + 1) / 2.0);
      j++;
    }

    /// Strain matrix S
    vpMatrix strainMatrix = cC[5]->run(vV, h);

    m_StrainMap.resize(strainMatrix.getRows(), strainMatrix.getCols());

    m_min_str = std::abs(strainMatrix.getMinValue());
    m_max_str = std::abs(strainMatrix.getMaxValue());
    m_max_abs = (m_min_str > m_max_str) ? m_min_str : m_max_str;
    for (unsigned int xIndex = 0; xIndex < strainMatrix.getCols(); ++xIndex) {
      for (unsigned int yIndex = 0; yIndex < strainMatrix.getRows(); ++yIndex) {
        m_StrainMap[yIndex][xIndex] =
            std::isnan(strainMatrix[yIndex][xIndex]) ? 0.0 : 254 * (fabs(strainMatrix[yIndex][xIndex]) / (m_max_abs));
      }
    }
  }

  return m_StrainMap;
}
#endif // QT
