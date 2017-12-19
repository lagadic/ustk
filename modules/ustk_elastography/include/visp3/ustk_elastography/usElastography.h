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

#ifndef USELASTOGRAPHY_H
#define USELASTOGRAPHY_H

#include <visp3/ustk_core/usConfig.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_OPENCV)

#include <QColor>
#include <QDebug>
#include <QElapsedTimer>
#include <QImage>
#include <QMutex>
#include <QReadWriteLock>
#include <QSharedPointer>
#include <QThread>
#include <QWaitCondition>

#include <visp3/core/vpImageTools.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_elastography/usConvolution2d.h>
#include <visp3/ustk_elastography/usMotionEstimation.h>
#include <visp3/ustk_elastography/usSignalProcessing.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

class VISP_EXPORT usElastography : public QThread
{
  Q_OBJECT
public:
  enum MotionEstimator { OF, BMA_TAYLOR };
  usElastography(bool t_isFiles = false);
  usElastography(usImageRF2D<short int> &Pre, usImageRF2D<short int> &Post);
  virtual ~usElastography();
  void setCommonSharedRFImage(QSharedPointer<usImageRF2D<short int> > t_RFIm);
  void setCommonSharedStrainImage(QSharedPointer<vpMatrix> t_StrainIm);
  void setPreCompression(usImageRF2D<short int> &Pre);
  void setPostCompression(usImageRF2D<short int> &Post);
  void setMotionEstimator(MotionEstimator t_mest) { m_mEstimatior = t_mest; }
  usImageRF2D<short int> getPreCompression(void);
  usImageRF2D<short int> getPostCompression(void);
  vpMatrix getStrainMap(void);
  void setLSQpercentage(double per);
  void setPRF(double prf);
  void setfs(double fs);
  double getfs(void) { return m_fs; }
  double getPRF(void) { return m_PRF; }
  // void run();
public slots:
  // void getImageStrain(void);
  void setPairRF(usImageRF2D<short int> Pre, usImageRF2D<short int> Post);
  void setRF(usImageRF2D<short int> t_RfArray);
  void setRF(void);
  void getCentroid(bool t_c);
  void reset(void) { m_Idx = 0; }
  virtual void run(void);
  void stop(void) { emit stopped(); }
  void setROI(int tx, int ty, int tw, int th);
  void updateROIPos(int tx, int ty);
  void useFiles(bool t_state);
signals:
  void StrainMapComp(void);
  void Centroid(QPointF t_c);
  void stopped(void);
  void roiSet(bool);

private:
  QMutex m_mutex;
  QWaitCondition m_cond;
  usImageRF2D<short int> m_receivedRF;
  vpMatrix m_StrainMap;
  usImageRF2D<short int> m_Precomp;
  usImageRF2D<short int> m_Postcomp;
  vpMatrix m_Elasto;
  double m_Lsqper;
  bool m_isloadPre;
  bool m_isloadPost;
  bool m_isStrainComp;
  bool m_isElastoComp;
  bool m_stopped;
  bool m_centroid;
  bool m_isElastoInitialized;
  bool m_isFiles;
  double m_PRF;
  double m_fs;
  double m_c;
  double m_min_str;
  double m_max_str;
  double m_max_abs;
  uint m_Idx;
  int m_h_m;
  int m_w_m;

  int m_ix;
  int m_iy;
  int m_rw;
  int m_rh;
  bool m_setROI;
  usImageRF2D<short int> m_PreROI;
  usImageRF2D<short int> m_PostROI;
  vpImagePoint m_ip;

  QVector<usConvolution2d *> cC;

  // Motion estimation
  MotionEstimator m_mEstimatior;
  usMotionEstimation m_ME;

  vpMatrix U;
  vpMatrix V;

  // Shared Pointers
  QSharedPointer<usImageRF2D<short int> > s_RFIm;
  QSharedPointer<vpMatrix> s_StrainIm;
  bool isSetSharedStrainMemory;

  QPointF GetImageCentroid(void);
};

#endif // QT && OPENCV
#endif // USELASTOGRAPHY_H
