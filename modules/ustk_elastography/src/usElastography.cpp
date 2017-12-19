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

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_OPENCV)

usElastography::usElastography(bool t_isFiles)
{
  qDebug("Elasto thread started");
  // Using default percentage of LSQ strain
  // If you want to change this value use
  // setLSQpercentage(double per);
  m_Lsqper = 0.01; // 0.08 BMA //Just OF 0.01
  // Frame rate in fps
  m_PRF = 24.0;
  // Sampling frequency
  m_fs = 40e6;
  // speed of the sound
  m_c = 1540.0;
  m_isloadPre = false;
  m_isloadPost = false;
  m_isStrainComp = false;
  m_isElastoComp = false;
  m_stopped = false;
  m_centroid = false;
  m_Idx = 0;
  m_setROI = false;
  // Using OF by default
  m_mEstimatior = OF;
  m_isElastoInitialized = false;
  m_isFiles = t_isFiles;

  // Avoiding leaking memory with the convolutions on cpu
  for (uint i = 0; i < 6; i++)
    cC.push_back(new usConvolution2d);

  isSetSharedStrainMemory = false;
}

usElastography::usElastography(usImageRF2D<short int> &Pre, usImageRF2D<short int> &Post)
{
  m_Precomp = Pre;
  m_Postcomp = Post;
  m_Lsqper = 0.01;
  m_PRF = 24.0;
  m_fs = 40e6;
  m_c = 1540.0;
  m_isloadPre = true;
  m_isloadPost = true;
  m_isStrainComp = false;
  m_isElastoComp = false;
  m_stopped = false;
  m_centroid = false;
  m_Idx = 0;
  m_setROI = false;
  m_mEstimatior = OF;
  m_isElastoInitialized = false;
  m_isFiles = false;

  // Avoiding leaking memory with the convolutions on cpu
  for (uint i = 0; i < 6; i++)
    cC.push_back(new usConvolution2d);

  isSetSharedStrainMemory = false;
}

usElastography::~usElastography()
{
  m_mutex.lock();
  m_stopped = true;
  emit stopped();
  m_cond.wakeOne();
  m_mutex.unlock();
}

inline void FindBlobs(const cv::Mat &binary, std::vector<std::vector<cv::Point2i> > &blobs)
{
  blobs.clear();

  // Fill the label_image with the blobs
  // 0  - background
  // 1  - unlabelled foreground
  // 2+ - labelled foreground

  cv::Mat label_image;
  binary.convertTo(label_image, CV_32SC1);

  int label_count = 2; // starts at 2 because 0,1 are used already

  for (int y = 0; y < label_image.rows; y++) {
    int *row = (int *)label_image.ptr(y);
    for (int x = 0; x < label_image.cols; x++) {
      if (row[x] != 1) {
        continue;
      }

      cv::Rect rect;
      cv::floodFill(label_image, cv::Point(x, y), label_count, &rect, 0, 0, 4);

      std::vector<cv::Point2i> blob;

      for (int i = rect.y; i < (rect.y + rect.height); i++) {
        int *row2 = (int *)label_image.ptr(i);
        for (int j = rect.x; j < (rect.x + rect.width); j++) {
          if (row2[j] != label_count) {
            continue;
          }

          blob.push_back(cv::Point2i(j, i));
        }
      }

      blobs.push_back(blob);

      label_count++;
    }
  }
}

QPointF usElastography::GetImageCentroid()
{
  vpImage<uchar> GrIm(m_StrainMap.getRows(), m_StrainMap.getCols());
  QPointF centroid(m_StrainMap.getCols() / 2.0, m_StrainMap.getRows() / 2.0);

  if (vpMath::abs(m_min_str - m_max_str) > (0.6 * m_max_abs)) {
    for (uint m = 0; m < GrIm.getCols(); m++)
      for (uint n = 0; n < GrIm.getRows(); n++) {
        double cVal = m_StrainMap.data[m * m_StrainMap.getRows() + n] * 255.0;
        GrIm(n, m, (uchar)cVal);
      }

    cv::Mat Image(GrIm.getRows(), GrIm.getCols(), CV_8UC1, GrIm.bitmap);
    cv::Mat binary;
    cv::threshold(Image, binary, 40, 1.0, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point2i> > blobs;
    FindBlobs(binary, blobs);
    cv::Mat output = cv::Mat::zeros(Image.size(), CV_8UC1);

    double m00, m01, m10, mold;
    m00 = m01 = m10 = mold = 0;
    // Randomy color the blobs
    for (size_t i = 0; i < blobs.size(); i++) {
      m00 = m01 = m10 = 0;

      for (size_t j = 0; j < blobs[i].size(); j++) {
        int x = blobs[i][j].x;
        int y = blobs[i][j].y;
        m00 += GrIm(y, x);
        m01 += y * GrIm(y, x);
        m10 += x * GrIm(y, x);
        output.at<uchar>(y, x) = GrIm(y, x);
      }
      if (m00 > mold) {
        centroid = QPointF(m10 / m00, m01 / m00);
        mold = m00;
      }
    }
  }
  return centroid;
}

void usElastography::getCentroid(bool t_c)
{
  m_mutex.lock();
  m_centroid = t_c;
  m_mutex.unlock();
}

void usElastography::setPreCompression(usImageRF2D<short int> &Pre)
{
  m_Precomp = Pre;
  m_isloadPre = true;
}

void usElastography::setPostCompression(usImageRF2D<short int> &Post)
{
  m_Postcomp = Post;
  m_isloadPost = true;
}

usImageRF2D<short int> usElastography::getPreCompression(void)
{
  assert(m_isloadPre);
  return m_Precomp;
}

usImageRF2D<short int> usElastography::getPostCompression(void)
{
  assert(m_isloadPost);
  return m_Postcomp;
}

vpMatrix usElastography::getStrainMap(void)
{
  assert(m_isStrainComp);
  return m_StrainMap;
}

void usElastography::setLSQpercentage(double per) { m_Lsqper = per; }

void usElastography::setPRF(double prf) { m_PRF = prf; }

void usElastography::setfs(double fs) { m_fs = fs; }

void usElastography::setPairRF(usImageRF2D<short int> Pre, usImageRF2D<short int> Post)
{
  m_mutex.lock();
  this->blockSignals(true);
  this->setPreCompression(Pre);
  this->setPostCompression(Post);
  this->blockSignals(false);
  m_mutex.unlock();
}

void usElastography::setRF(usImageRF2D<short int> t_RfArray)
{
  if (m_Idx == 0)
    setPreCompression(t_RfArray);
  else {
    if (m_Idx > 1)
      setPreCompression(m_Postcomp);
    setPostCompression(t_RfArray);
  }
  m_Idx++;
}

void usElastography::setRF()
{
  if (m_Idx == 0)
    setPreCompression(*(s_RFIm.data()));
  else {
    if (m_Idx > 1)
      setPreCompression(m_Postcomp);
    setPostCompression(*(s_RFIm.data()));
  }
  m_Idx++;
}

void usElastography::setROI(int tx, int ty, int tw, int th)
{
  m_ix = tx;
  m_iy = ty;
  m_rw = tw;
  m_rh = th;
  m_ip.set_i(m_iy);
  m_ip.set_j(m_ix);
  m_setROI = true;
  if (m_isFiles) {
    m_isloadPre = true;
    m_isloadPost = true;
  }
  emit roiSet(true);
}

void usElastography::updateROIPos(int tx, int ty)
{
  m_ix = tx;
  m_iy = ty;
}

void usElastography::useFiles(bool t_state) { m_isFiles = t_state; }

void usElastography::setCommonSharedRFImage(QSharedPointer<usImageRF2D<short int> > t_RFIm) { s_RFIm = t_RFIm; }

void usElastography::setCommonSharedStrainImage(QSharedPointer<vpMatrix> t_StrainIm)
{
  isSetSharedStrainMemory = true;
  s_StrainIm = t_StrainIm;
}

inline usImageRF2D<short int> usImageRF_ROI(const usImageRF2D<short int> &M, uint r, uint c, uint nrows, uint ncols)
{
  uint rnrows = r + nrows;
  uint cncols = c + ncols;
  usImageRF2D<short int> t_Mout;
  assert(rnrows < M.getRows() || cncols < M.getCols());
  t_Mout.resize(nrows, ncols);
  for (unsigned int i = c; i < cncols; i++)
    for (unsigned int j = r; j < rnrows; j++)
      t_Mout.bitmap[(i - c) * nrows + (j - r)] = M.bitmap[i * M.getRows() + j];

  return t_Mout;
}

void usElastography::run()
{
  std::cout << "thread elasto " << this->thread()->currentThreadId() << std::endl;
  forever
  {
    if (m_stopped) {
      qDebug("Stoping thread Elasto ");
      emit stopped();
      // break;
    }
    if (m_isloadPre == true && m_isloadPost == true && m_setROI == true) {
      // cout << "Pre or Post frames loaded" << endl;
      /*vpImageTools::crop<short int>(m_Precomp, m_iy, m_ix, m_rh, m_rw, m_PreROI);
      vpImageTools::crop<short int>(m_Postcomp, m_iy, m_ix, m_rh, m_rw, m_PostROI);*/
      m_PreROI = usImageRF_ROI(m_Precomp, m_iy, m_ix, m_rh, m_rw);
      m_PostROI = usImageRF_ROI(m_Postcomp, m_iy, m_ix, m_rh, m_rw);
      assert(m_PreROI.getCols() == m_PostROI.getCols());
      assert(m_PreROI.getRows() == m_PostROI.getRows());
      if (m_mEstimatior == BMA_TAYLOR) {
        // Step 0: BMA
        m_ME.init(m_PreROI, m_PostROI, 2, 20, 2, 120);
        m_ME.run();
        // m_ME.saveV("dispV.dat");
        U = m_ME.getU_vp() * (m_c * (m_PRF / (2.0 * m_fs)));
        V = m_ME.getV_vp() * (m_c * (m_PRF / (2.0 * m_fs)));
        m_h_m = m_PreROI.getRows();
        m_w_m = m_PreROI.getCols();
      } else {
        // Step 1: Numerical gradients
        vpMatrix Fx = usSignalProcessing::GetGx(m_PreROI);
        vpMatrix Fy = usSignalProcessing::GetGy(m_PreROI);
        // Difference
        // vpMatrix Dt = m_Postcomp-m_Precomp;
        vpMatrix Dt = usSignalProcessing::Difference(m_PostROI, m_PreROI);
        // Step 2: computing the quadratic functions
        vpMatrix dx2 = usSignalProcessing::HadamardProd(Fx, Fx);
        vpMatrix dy2 = usSignalProcessing::HadamardProd(Fy, Fy);
        vpMatrix dxy = usSignalProcessing::HadamardProd(Fx, Fy);
        vpMatrix dtx = usSignalProcessing::HadamardProd(Dt, Fx);
        vpMatrix dty = usSignalProcessing::HadamardProd(Dt, Fy);
        // Step 3: blur the 5 images
        vpMatrix h_gauss = usSignalProcessing::GaussianFilter(15, 5, 7.5); //(51,35,5); //21 good-phantom

        if (m_isElastoInitialized) {
          cC[0]->update(dx2, h_gauss);
          cC[1]->update(dy2, h_gauss);
          cC[2]->update(dxy, h_gauss);
          cC[3]->update(dty, h_gauss);
          cC[4]->update(dtx, h_gauss);
        } else {
          cC[0]->init(dx2, h_gauss);
          cC[1]->init(dy2, h_gauss);
          cC[2]->init(dxy, h_gauss);
          cC[3]->init(dty, h_gauss);
          cC[4]->init(dtx, h_gauss);
        }
        cC[0]->run();
        cC[1]->run();
        cC[2]->run();
        cC[3]->run();
        cC[4]->run();

        vpMatrix gdx2 = cC[0]->getConvolution();
        vpMatrix gdy2 = cC[1]->getConvolution();
        vpMatrix gdxy = cC[2]->getConvolution();
        vpMatrix gdty = cC[3]->getConvolution();
        vpMatrix gdtx = cC[4]->getConvolution();

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

        U.resize(h_w, w_w);
        V.resize(h_w, w_w);
        vpMatrix b(2, 1);
        vpMatrix M(2, 2);
        vpMatrix X(2, 1);
        uint k = 0, l;
        for (uint m = 0; m < (uint)(m_h_m - Wsizey - Wincy); m += Wincy) {
          l = 0;
          for (uint n = 0; n < (uint)(m_w_m - Wsizex - Wincx); n += Wincx) {
            double sgdx2_w = 0.0;
            double sgdy2_w = 0.0;
            double sgdxy_w = 0.0;
            double sgdtx_w = 0.0;
            double sgdty_w = 0.0;
            for (uint i = m; i < (m + Wsizey); i++) {
              for (uint j = n; j < (n + Wsizex); j++) {
                sgdx2_w += gdx2.data[j * m_h_m + i];
                sgdy2_w += gdy2.data[j * m_h_m + i];
                sgdxy_w += gdxy.data[j * m_h_m + i];
                sgdtx_w += gdtx.data[j * m_h_m + i];
                sgdty_w += gdty.data[j * m_h_m + i];
              }
            }
            M.data[0] = sgdx2_w;
            M.data[1] = sgdxy_w;
            M.data[2] = sgdxy_w;
            M.data[3] = sgdy2_w;
            b.data[0] = -sgdtx_w;
            b.data[1] = -sgdty_w;
            X = M.pseudoInverse() * b;
            U.data[k + l * h_w] = X.data[0] * (m_c * (m_PRF / (2.0 * m_fs)));
            V.data[k + l * h_w] = X.data[1] * (m_c * (m_PRF / (2.0 * m_fs)));
            l++;
          }
          k++;
        }
      }

      vpMatrix vV(m_h_m, m_w_m, true);
      vV = usSignalProcessing::BilinearInterpolation(V, m_w_m, m_h_m);
      /// Strain estimation
      /// "LSQ strain estimation"
      double kappa = (2.0 * m_fs) / (m_c * m_PRF);
      double d_m = m_Lsqper * m_h_m; // 20.0; //Good value in phantom
      double n = d_m + 1;
      // Filter kernel
      vpMatrix h((uint)n + 1, 1, true);
      double xi = kappa * 12.0 / (n * (n * n - 1));
      uint j = 0;
      for (int i = n; i >= 0; i--) {
        h.data[j] = xi * (i - (n + 1) / 2.0);
        j++;
      }

      if (m_isElastoInitialized)
        cC[5]->update(vV, h);
      else
        cC[5]->init(vV, h);
      cC[5]->run();
      /// Strain matrix S
      m_StrainMap = cC[5]->getConvolution();

      m_min_str = qAbs(m_StrainMap.getMinValue());
      m_max_str = qAbs(m_StrainMap.getMaxValue());
      m_max_abs = (m_min_str > m_max_str) ? m_min_str : m_max_str;
      for (uint i = 0; i < m_StrainMap.size(); i++) {
        *(m_StrainMap.data + i) =
            std::isnan(*(m_StrainMap.data + i)) ? 0.0 : fabs(*(m_StrainMap.data + i)) / (m_max_abs);
        //(*(m_StrainMap.data + i) - min_str)/(max_abs);
      }
      if (isSetSharedStrainMemory)
        *(s_StrainIm.data()) = m_StrainMap;

      m_isStrainComp = true;
      if (!m_isFiles) {
        m_isloadPre = false;
        m_isloadPost = false;
      }
      // std::cout <<"Achieved " << std::endl;
      emit StrainMapComp();
      if (m_centroid)
        emit Centroid(this->GetImageCentroid());
      m_isElastoInitialized = true;
    }
  }
}
#endif // QT && OPENCV
