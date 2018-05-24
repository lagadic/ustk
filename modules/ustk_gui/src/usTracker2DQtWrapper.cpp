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

/**
* @file usTracker2DQtWrapper.cpp
* @brief Qt wrapper for usDenseTracker2D class.
*/

#include <visp3/ustk_gui/usTracker2DQtWrapper.h>

#if (defined(USTK_HAVE_VTK_QT) || defined(USTK_HAVE_QT5)) && defined(VISP_HAVE_MODULE_USTK_TEMPLATE_TRACKING)

/**
* Constructor.
*/
usTracker2DQtWrapper::usTracker2DQtWrapper()
  : QObject(), m_tracker(), m_firstImage(), m_firstFrameArrived(false), m_isInitialized(false)
{
}

/**
* Destructor.
*/
usTracker2DQtWrapper::~usTracker2DQtWrapper() {}

/**
* Slot to init the tracker.
* @param rect Rectangle to track, expressed in vpImage dimentions.
*/
void usTracker2DQtWrapper::initTracker(vpRectOriented rect)
{
  if (m_firstFrameArrived) {
    m_tracker.init(m_firstImage, rect);
    m_isInitialized = true;
  } else {
    throw(vpException(vpException::fatalError, "Cannot init tracker: no frames sent to usTracker2DQtWrapper."));
  }
}

/**
* Slot to update the image for the tracker.
* @param image New image input for tracking.
*/
void usTracker2DQtWrapper::updateImage(vpImage<unsigned char> image)
{
  if (m_isInitialized) {
    m_tracker.update(image);
    emit(newTrackedRectangle(m_tracker.getTarget()));
  } else {
    m_firstImage = image;
    m_firstFrameArrived = true;
  }
}

/**
* Slot to update the image for the tracker in case of pre-scan image sent.
* @param image New image input for tracking.
*/
void usTracker2DQtWrapper::updateImage(usImagePreScan2D<unsigned char> image)
{
  if (m_isInitialized) {
    m_tracker.update(image);
    emit(newTrackedRectangle(m_tracker.getTarget()));
  } else {
    m_firstImage = image;
    m_firstFrameArrived = true;
  }
}

/**
* Slot to update the image for the tracker in case of post-scan image sent.
* @param image New image input for tracking.
*/
void usTracker2DQtWrapper::updateImage(usImagePostScan2D<unsigned char> image)
{
  if (m_isInitialized) {
    m_tracker.update(image);
    emit(newTrackedRectangle(m_tracker.getTarget()));
  } else {
    m_firstImage = image;
    m_firstFrameArrived = true;
  }
}

/**
* Slot to stop the tracking process.
*/
void usTracker2DQtWrapper::stopTracking()
{
  m_firstFrameArrived = false;
  m_isInitialized = false;
}
#endif
