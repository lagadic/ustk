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

/**
 * @file usDenseTracker2D.h
 * @brief 2D region tracker
 */

#ifndef __usDenseTracker2D_h_
#define __usDenseTracker2D_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpRectOriented.h>

/**
 * @class usDenseTracker2D
 * @brief 2D region tracker
 * @ingroup module_ustk_template_tracking
 *
 * Class to perform a 2D tracking of a region of interest.
 *
 * See \cite Nadeau15a for more details.
 *
 */
class VISP_EXPORT usDenseTracker2D
{
public:
  usDenseTracker2D();
  ~usDenseTracker2D();

  vpImage<unsigned char> &getRegion();

  vpRectOriented getTarget() const;

  vpImage<unsigned char> &getTemplate();

  void init(const vpImage<unsigned char> &I, const vpRectOriented &R);

  bool isInit();

  void update(const vpImage<unsigned char> &I);

private:
  vpColVector s_desired;
  vpColVector s_current;
  vpRectOriented m_target;
  vpImage<unsigned char> m_template;
  vpImage<unsigned char> m_region;
  vpImage<double> m_gradX;
  vpImage<double> m_gradY;
  vpMatrix m_LI;
  vpMatrix m_LI_inverse;
  unsigned int m_height;
  unsigned int m_width;
  unsigned int m_size;
  bool m_isInit;
};

#endif // US_DENSE_TRACKER_2D_H
