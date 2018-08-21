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
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_elastography/usElastography3D.h>

#if defined(USTK_HAVE_FFTW)

/**
* Default constructor.
*/
usElastography3D::usElastography3D()
{
  m_isloadPre = false;
  m_isloadPost = false;
  m_setROI = false;
  m_framesInROI = 0;
}

/**
* Constructor setting the pre-compressed and the post-compressed volumes to perform elastography on.
*
* @param Pre Pre-compresssed RF volume.
* @param Post Post-compresssed RF volume.
*/
usElastography3D::usElastography3D(usImageRF3D<short int> &Pre, usImageRF3D<short int> &Post)
{
  setPreCompression(Pre);
  setPostCompression(Post);

  m_setROI = false;
}

/**
* Destructor.
*/
usElastography3D::~usElastography3D() {}

/**
* Pre-compresssed RF volume setter.
* @param Pre Pre-compresssed RF volume.
*/
void usElastography3D::setPreCompression(const usImageRF3D<short int> &Pre)
{
  m_Precomp = Pre;
  m_isloadPre = true;
}

/**
* Post-compresssed RF volume setter.
* @param Post Post-compresssed RF volume.
*/
void usElastography3D::setPostCompression(const usImageRF3D<short int> &Post)
{
  m_Postcomp = Post;
  m_isloadPost = true;
}

/**
* Setter for LSQ strain percentage.
* @param per LSQ strain percentage.
*/
void usElastography3D::setLSQpercentage(double per) { m_elastography2DProcessor.setLSQpercentage(per); }

/**
* Setter for LSQ strain percentage.
* @param fps LSQ strain percentage.
*/
void usElastography3D::setFPS(double fps) { m_elastography2DProcessor.setFPS(fps); }

/**
* Setter for sampling frequency of the ultrasound wave.
* @param fs Sampling frequency in Hertz.
*/
void usElastography3D::setSamplingFrequency(double samplingFrequency)
{
  m_elastography2DProcessor.setSamplingFrequency(samplingFrequency);
}

void usElastography3D::setMotionEstimator(usElastography::MotionEstimator t_mest)
{
  m_elastography2DProcessor.setMotionEstimator(t_mest);
}

double usElastography3D::getFPS(void) { return m_elastography2DProcessor.getFPS(); }

double usElastography3D::getSamplingFrequency(void) { return m_elastography2DProcessor.getSamplingFrequency(); }

void usElastography3D::setDecimationFactor(unsigned int decimationFactor)
{
  m_elastography2DProcessor.setDecimationFactor(decimationFactor);
}

/**
* Updater of pre/post compressed volumes : sets the new volume as pre-compressed volume for process, and sets the old
* pre-compressed volume as new post-compressed.
* @param volume New pre-compressed volume.
*/
void usElastography3D::updateRF(const usImageRF3D<short> &volume)
{
  if (!m_isloadPre) {
    setPreCompression(volume);
  } else {
    if (m_isloadPost)
      setPreCompression(m_Postcomp);
    setPostCompression(volume);
  }
}

/**
* ROI setter: the ROI is the region in the RF image in which to compute the elastography.
* @param tx Top left column of the ROI.
* @param ty Top left row of the ROI.
* @param tz Top left frame of the ROI.
* @param tw ROI width in px.
* @param th ROI height in px.
* @param tf ROI size in number of frames included.
*/
void usElastography3D::setROI(int tx, int ty, int tz, int tw, int th, int tf)
{
  m_elastography2DProcessor.setROI(tx, ty, tw, th);
  m_setROI = true;
  m_framesInROI = tf;
  m_frameBeginROI = tz;
}

/**
* ROI position update, allows to move the ROI in the image.
* @param tx New top left column of the ROI.
* @param ty New top left row of the ROI.
* @param ty New top left frame of the ROI.
*/
void usElastography3D::updateROIPos(int tx, int ty, int tz)
{
  m_elastography2DProcessor.updateROIPos(tx, ty);
  m_frameBeginROI = tz;
}

/**
* Run the elastography computation.
* @return The elastography image of the ROI (dark = hard tissues, white = soft).
*/
usImage3D<unsigned char> usElastography3D::run()
{
  usImageRF2D<short int> preCompressedFrame, postCompressedFrame;
  usImage3D<unsigned char> outputVolume;
  vpImage<unsigned char> outputFrame;

  if (m_isloadPre == true && m_isloadPost == true && m_setROI == true) {
    for (int i = m_frameBeginROI; i < m_frameBeginROI + m_framesInROI; i++) {
      m_Precomp.getFrame(preCompressedFrame, i);
      m_Postcomp.getFrame(postCompressedFrame, i);
      m_elastography2DProcessor.setPreCompression(preCompressedFrame);
      m_elastography2DProcessor.setPostCompression(postCompressedFrame);

      outputFrame = m_elastography2DProcessor.run();
      outputVolume.resize(outputFrame.getHeight(), outputFrame.getWidth(), m_framesInROI);
      for (unsigned int row = 0; row < outputFrame.getHeight(); row++) {
        for (unsigned int col = 0; col < outputFrame.getWidth(); col++) {
          outputVolume(row, col, i - m_frameBeginROI, outputFrame[row][col]);
        }
      }
    }
  }

  return outputVolume;
}
#endif // QT
