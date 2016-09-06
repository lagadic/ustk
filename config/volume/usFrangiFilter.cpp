/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

#include "usFrangiFilter.h"

#include <UsTk/usVolumeProcessing.h>

#include <visp/vpColVector.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>

usFrangiFilter::usFrangiFilter() {}

void usFrangiFilter::init()
{
  m_a = 1.0;
  m_b = 1.0;
  m_c = 1.0;
  m_dimx = 0;
  m_dimy = 0;
  m_dimz = 0;
  m_minScale = 0.0;
  m_maxScale = 0.0;
  m_Fxx = usVolumeProcessing::getDerivativeFilterXX(1.0, 3);
  m_Fyy = usVolumeProcessing::getDerivativeFilterYY(1.0, 3);
  m_Fzz = usVolumeProcessing::getDerivativeFilterZZ(1.0, 3);
  m_Fxy = usVolumeProcessing::getDerivativeFilterXY(1.0, 3);
  m_Fxz = usVolumeProcessing::getDerivativeFilterXZ(1.0, 3);
  m_Fyz = usVolumeProcessing::getDerivativeFilterYZ(1.0, 3);
}

void usFrangiFilter::setInput(usVolume<unsigned char> *Src)
{
  // Resize data containers if necessary
  if ((m_dimx != Src->getDimX())
      || (m_dimy != Src->getDimY())
      || (m_dimz != Src->getDimZ()))
    {
      m_dimx = Src->getDimX();
      m_dimy = Src->getDimY();
      m_dimz = Src->getDimZ();
      m_Hxx.resize(m_dimx, m_dimy, m_dimz);
      m_Hyy.resize(m_dimx, m_dimy, m_dimz);
      m_Hzz.resize(m_dimx, m_dimy, m_dimz);
      m_Hxy.resize(m_dimx, m_dimy, m_dimz);
      m_Hxz.resize(m_dimx, m_dimy, m_dimz);
      m_Hyz.resize(m_dimx, m_dimy, m_dimz);
      m_output.resize(m_dimx, m_dimy, m_dimz);
      m_available.resize(m_dimx, m_dimy, m_dimz);
    }
  
  // Reset availability to false for all voxels
  m_available.initData(false);

  // Store reference to input data
  m_input = Src;
}

double usFrangiFilter::getOutput(unsigned int x, unsigned int y, unsigned int z)
{
  if (!m_available(x, y, z)) {
    m_output(x, y, z, computeFrangi(x, y, z));
    m_available(x, y, z, true);
  }
  return m_output(x, y, z);
}

double usFrangiFilter::computeFrangi(unsigned int x, unsigned int y, unsigned int z)
{
  double foo = usVolumeProcessing::applyFilter(*m_input, m_Fxx, x, y, z);
  m_Hxx(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fxx, x, y, z));
  m_Hyy(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fyy, x, y, z));
  m_Hzz(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fzz, x, y, z));
  m_Hxy(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fxy, x, y, z));
  m_Hxz(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fxz, x, y, z));
  m_Hyz(x, y, z, usVolumeProcessing::applyFilter(*m_input, m_Fyz, x, y, z));
  vpMatrix H(3,3);
  vpColVector evalues;
  double Ra, Rb, S, v;

  H[0][0] = m_Hxx(x, y, z); H[0][1] = m_Hxy(x, y, z); H[0][2] = m_Hxz(x, y, z);
  H[1][0] = m_Hxy(x, y, z); H[1][1] = m_Hyy(x, y, z); H[1][2] = m_Hyz(x, y, z);
  H[2][0] = m_Hxz(x, y, z); H[2][1] = m_Hyz(x, y, z); H[2][2] = m_Hzz(x, y, z);

  evalues = H.eigenValues();

  // Sort eigenvalues
  if (vpMath::abs(evalues[0]) > vpMath::abs(evalues[1]))
    std::swap(evalues[0], evalues[1]);
  if (vpMath::abs(evalues[1]) > vpMath::abs(evalues[2]))
    std::swap(evalues[1], evalues[2]);
  if (vpMath::abs(evalues[0]) > vpMath::abs(evalues[1]))
    std::swap(evalues[0], evalues[1]);

  if ((evalues[1] >= 0.0) || (evalues[2] >= 0.0))
    v = 0.0;
  else {
    Rb = vpMath::abs(evalues[0])
      / sqrt(vpMath::abs(evalues[1] * evalues[2]));
	  
    Ra = vpMath::abs(evalues[1] / evalues[2]);
	  
    S = evalues.euclideanNorm();
	  
    v = (1.0 - exp(- vpMath::sqr(Ra) / (2.0 * vpMath::sqr(m_a))))
      * exp(- vpMath::sqr(Rb) / (2.0 * vpMath::sqr(m_b)))
      * (1.0 - exp(- vpMath::sqr(S) / (2.0 * vpMath::sqr(m_c))));
  }
  return v;
}
