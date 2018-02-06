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
 * @file usVolumeGrabbedInfo.h
 * @brief Class to store additionnal informations arriving on the network with ultrasound volume grabbed (volume count,
 * timestamps ...).
 */

#ifndef __usVolumeGrabbedInfo_h_
#define __usVolumeGrabbedInfo_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <visp3/ustk_grabber/usNetworkGrabber.h>

#include <QtCore/QTypeInfo>

/**
 * @class usVolumeGrabbedInfo
 * @brief Class to store additionnal informations arriving on the network with ultrasound volumes grabbed, such as
 * volume count, timestamps.
 * Usefull to do real-time process.
 * @ingroup module_ustk_grabber
 */
template <class Type> class usVolumeGrabbedInfo : public Type
{
public:
  explicit usVolumeGrabbedInfo();
  ~usVolumeGrabbedInfo();

  void addTimeStamp(quint64 timestamp, unsigned int position);

  quint32 getVolumeCount() const;
  std::vector<uint64_t> getTimeStamps() const;

  void setVolumeCount(quint32 volumeCount);

private:
  quint32 m_volumeCount;              // from the beginning of acquisition
  std::vector<uint64_t> m_timestamps; // msecs since epoch (on ultrasond machine)
};

/**
* Constructor.
*/
template <class Type> usVolumeGrabbedInfo<Type>::usVolumeGrabbedInfo() : Type(), m_volumeCount(0), m_timestamps() {}

/**
* Destructor.
*/
template <class Type> usVolumeGrabbedInfo<Type>::~usVolumeGrabbedInfo() {}

/**
* Volume count getter.
* @return The volume number since beginning of acquisition.
*/
template <class Type> quint32 usVolumeGrabbedInfo<Type>::getVolumeCount() const { return m_volumeCount; }

/**
* Volume count setter.
* @param volumeCount The volume number since beginning of acquisition.
*/
template <class Type> void usVolumeGrabbedInfo<Type>::setVolumeCount(quint32 volumeCount)
{
  m_volumeCount = volumeCount;
}

/**
* Includes a new timestamp into the array (corresponding to a frame of the volume).
* @param timestamp The new timestamp when the first frame in the volume was acquired.
* @param position The new timestamp position in the array (from O to array size - 1).
*/
template <class Type> void usVolumeGrabbedInfo<Type>::addTimeStamp(quint64 timestamp, unsigned int position)
{
  if (position == m_timestamps.size())
    m_timestamps.push_back(timestamp);
  else if (position > m_timestamps.size()) {
    m_timestamps.resize(position + 1);
    m_timestamps.at(position) = timestamp;
  } else
    m_timestamps.at(position) = timestamp;
}

/**
* Getter for the timestamps array.
* @return The vector of timestamps (make sure you filled it with addTimeStamp() at every new frame before call this
* method.
*/
template <class Type> std::vector<uint64_t> usVolumeGrabbedInfo<Type>::getTimeStamps() const { return m_timestamps; }

/*!
  Print volume grabbed information in a ostream.
  Usage example:
  \code
  usVolumeGrabbedInfo<Type> myVolumeGrabbed;
  std::cout << myVolumeGrabbed << std::endl;
  \endcode
*/
template <class Type> std::ostream &operator<<(std::ostream &out, const usVolumeGrabbedInfo<Type> &other)
{
  out << "volumeCount : " << other.getVolumeCount() << std::endl << (Type)other << std::endl;

  return out;
}

#endif // QT4 || QT5
#endif // __usVolumeGrabbedInfo_h_
