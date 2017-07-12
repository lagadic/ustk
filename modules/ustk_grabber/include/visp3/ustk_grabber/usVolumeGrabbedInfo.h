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
 * @brief Class to store additionnal informations arriving on the network with ultrasound volume grabbed (volume count, timestamps ...).
 */

#ifndef __usVolumeGrabbedInfo_h_
#define __usVolumeGrabbedInfo_h_


#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_QT5) || defined(USTK_HAVE_VTK_QT)

#include <visp3/ustk_grabber/usNetworkGrabber.h>

#include <QtCore/QTypeInfo>

/**
 * @class usVolumeGrabbedInfo
 * @brief Class to store additionnal informations arriving on the network with ultrasound volumes grabbed, such as volume count, timestamps.
 * Usefull to do real-time process.
 * @ingroup module_ustk_grabber
 */
template<class Type>
class usVolumeGrabbedInfo : public Type
{
public:

  explicit usVolumeGrabbedInfo();
  ~usVolumeGrabbedInfo();

  quint32 getVolumeCount() const;
  quint64 getFistFrameTimeStamp() const;
  quint64 getLastFrameTimeStamp() const;

  void setVolumeCount(quint32 volumeCount);
  void setFirstFrameTimeStamp(quint64 firstFrameTimeStamp);
  void setLastFrameTimeStamp(quint64 timeStamp);

private:
  quint32 m_volumeCount; //from the beginning of acquisition
  quint64 m_firstFrameTimeStamp; //msecs since epoch (on ultrasond machine) for first frame of the volume
  quint64 m_lastFrameTimeStamp; //msecs since epoch (on ultrasond machine) for last frame of the volume
};

/**
* Constructor.
*/
template<class Type>
usVolumeGrabbedInfo<Type>::usVolumeGrabbedInfo() :
  Type(), m_volumeCount(0), m_firstFrameTimeStamp(0), m_lastFrameTimeStamp(0)
{

}

/**
* Destructor.
*/
template<class Type>
usVolumeGrabbedInfo<Type>::~usVolumeGrabbedInfo()
{

}

/**
* Volume count getter.
* @return The volume number since beginning of acquisition.
*/
template<class Type>
quint32 usVolumeGrabbedInfo<Type>::getVolumeCount() const {
  return m_volumeCount;
}

/**
* Volume first timestamp getter.
* @return The timestamp when the first frame in the volume was acquired.
*/
template<class Type>
quint64 usVolumeGrabbedInfo<Type>::getFistFrameTimeStamp() const{
  return m_firstFrameTimeStamp;
}

/**
* Volume last timestamp getter.
* @return The timestamp when the last frame in the volume was acquired.
*/
template<class Type>
quint64 usVolumeGrabbedInfo<Type>::getLastFrameTimeStamp() const{
  return m_lastFrameTimeStamp;
}

/**
* Volume count setter.
* @param volumeCount The volume number since beginning of acquisition.
*/
template<class Type>
void usVolumeGrabbedInfo<Type>::setVolumeCount(quint32 volumeCount) {
  m_volumeCount = volumeCount;
}

/**
* Volume first timestamp setter.
* @param firstFrameTimeStamp The timestamp when the first frame in the volume was acquired.
*/
template<class Type>
void usVolumeGrabbedInfo<Type>::setFirstFrameTimeStamp(quint64 firstFrameTimeStamp){
  m_firstFrameTimeStamp = firstFrameTimeStamp;
}

/**
* Volume last timestamp setter.
* @param lastFrameTimeStamp The timestamp when the last frame in the volume was acquired.
*/
template<class Type>
void usVolumeGrabbedInfo<Type>::setLastFrameTimeStamp(quint64 lastFrameTimeStamp){
  m_lastFrameTimeStamp = lastFrameTimeStamp;
}

/*!
  Print volume grabbed information in a ostream.
  Usage example:
  \code
  usVolumeGrabbedInfo<Type> myVolumeGrabbed;
  std::cout << myVolumeGrabbed << std::endl;
  \endcode
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usVolumeGrabbedInfo<Type> &other)
{
  out << "volumeCount : " << other.getTimeStamp() << std::endl
      << "firstFrameTimeStamp : " << other.getFistFrameTimeStamp() << std::endl
      << "lastFrameTimeStamp per volume : " << other.getLastFrameTimeStamp() << std::endl
      << (Type)other << std::endl;

  return out;
}

#endif // QT4 || QT5
#endif // __usVolumeGrabbedInfo_h_
