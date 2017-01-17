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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImagePostScan2D.h
 * @brief 2D post-scan ultrasound image.
 */

#ifndef __usImagePostScan2D_h_
#define __usImagePostScan2D_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usTransducerSettings.h>

/**
  @class usImagePostScan2D
  @brief 2D post-scan ultrasound image.
  @ingroup module_ustk_core

  This class represents a 2D post-scan ultrasound image. This image is nothing more than a vpImage that
  contains ultrasound 2D post-scan data and additional settings that give information about the
  acquisition process done by the transducer.

  The settings associated to an usImagePostScan2D image are the following:
  - the transducer settings (see usTransducerSettings) that are:
    - the name of the probe that could be set using setProbeName() or retrieved using getProbeName().
    - the transducer radius \f$R_{_T}\f$ in meters (value set to zero for a linear transducer).
      Its value could be set using setTransducerRadius() and retrieved using getTransducerRadius().
    - the scan line pitch that corresponds to the angle \f$\alpha_{_{SC}}\f$ (in radians) between
      two successive scan line beams when the transducer is convex, or to the distance \f$d_{_{SC}}\f$
      (in meters) when the transducer is linear. To set this value use setScanLinePitch() and to get
      its value use getScanLinePitch().
    - the number of scan lines \f$n_{_{SC}}\f$. To set this setting use setScanLineNumber() and to access
      to the value use getScanLineNumber().
    - the type of ultrasound transducer used for data acquisition: convex or linear. This parameter
      could be set using setTransducerConvexity(). To know the transducer type use isTransducerConvex().
    - the depth that corresponds to the distance in meters between the first and the last pixel in a scan line.
      To set this value use setDepth() and to get the depth use getDepth().
    .
  - and two additional resolution parameters called \f$p_x, p_y\f$ that correspond to the size in meter
    of a pixel. They can be set using setWidthResolution() and setHeightResolution() respectively. To access
    to the values use getWidthResolution() and getHeightResolution() respectively.

  The following figure summarize these settings and shows the structure of an usImagePostScan2D image:
  \image html img-usImagePostScan2D.png

  The following example shows how to build a 2D post-scan ultrasound image from a vpImage and from acquisition settings.

  \code
#include <visp3/ustk_core/usImagePostScan2D.h>

int main()
{
  // 2D post-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double widthResolution = 0.002;
  double heightResolution = 0.002;

  vpImage<unsigned char> I(height, width);
  usImagePostScan2D<unsigned char> postScan2d;
  postScan2d.setTransducerRadius(transducerRadius);
  postScan2d.setScanLinePitch(scanLinePitch);
  postScan2d.setScanLineNumber(scanLineNumber);
  postScan2d.setTransducerConvexity(isTransducerConvex);
  postScan2d.setWidthResolution(widthResolution);
  postScan2d.setHeightResolution(heightResolution);
  postScan2d.setData(I);
}
  \endcode
 */
template<class Type>
class usImagePostScan2D : public vpImage<Type>, public usTransducerSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(const vpImage<Type> &image, const usTransducerSettings &transducerSettings,
                    double widthResolution = 0.0, double heightResolution = 0.0);
  usImagePostScan2D(const usImagePostScan2D<Type> &other);
  virtual ~usImagePostScan2D();

  double getHeightResolution() const;
  double getWidthResolution() const;

  usImagePostScan2D<Type> & operator =(const usImagePostScan2D<Type> &other);
  bool operator ==(const usImagePostScan2D<Type> &other);

  void setData(const vpImage<Type> &image);
  void setHeightResolution(double heightResolution);
  void setWidthResolution(double widthResolution);

private:
  double m_widthResolution;
  double m_heightResolution;
};

/**
* Basic constructor, all parameters set to default values.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D()
  : vpImage<Type>(), usTransducerSettings(), m_widthResolution(0.0), m_heightResolution(0.0)
{

}

/**
* Copy constructor from an other 2D post-scan image.
* @param other 2D post-scan image to copy.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const usImagePostScan2D<Type> &other)
: vpImage<Type>(other), usTransducerSettings(other), 
  m_widthResolution(other.getWidthResolution()), m_heightResolution(other.getHeightResolution())
{

}

/**
* Constructor from an image and transducer settings.
* @param image Image containing the 2D post-scan data to copy.
* @param transducerSettings Transducer settings associated to the data.
* @param heightResolution Height (in meters) of a pixel.
* @param widthResolution Width (in meters) of a pixel.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const vpImage<Type> &image,
                                           const usTransducerSettings &transducerSettings,
                                           double widthResolution, double heightResolution)
: vpImage<Type>(image), usTransducerSettings(transducerSettings),
  m_widthResolution(widthResolution), m_heightResolution(heightResolution)
{

}

/**
* Destructor.
*/
template<class Type>
usImagePostScan2D<Type>::~usImagePostScan2D() {}

/**
* Assignement operator.
*/
template<class Type>
usImagePostScan2D<Type> & usImagePostScan2D<Type>::operator =(const usImagePostScan2D<Type> &other)
{
  //from vpImage
  vpImage<Type>::operator=(other);

  //from usTransducerSettings
  usTransducerSettings::operator=(other);

  //from this class
  m_heightResolution = other.getHeightResolution();
  m_widthResolution = other.getWidthResolution();

  return *this;
}

/**
* Comparison operator.
*/
template<class Type>
bool usImagePostScan2D<Type>::operator ==(const usImagePostScan2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) &&
      usTransducerSettings::operator== (other) &&
      m_heightResolution == other.getHeightResolution() &&
      m_widthResolution == other.getWidthResolution());
}

/**
* Operator to print 2D post-scan image information on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePostScan2D<Type> &other)
{
  return out << static_cast<const usTransducerSettings &>(other)
             << "image width : " << other.getWidth() << std::endl
             << "image height : " << other.getHeight() << std::endl
             << "image width resolution : " << other.getWidthResolution()
             << "image height resolution : " << other.getHeightResolution() << std::endl;
}

/**
* Setter that updates 2D post-scan image data.
* @param image Data to set.
*/
template<class Type>
void usImagePostScan2D<Type>::setData(const vpImage<Type> &image)
{
  vpImage<Type>::operator=(image);
}

/**
* Getter for pixel height resolution.
* @return Height of a pixel (in meters).
*/
template<class Type>
double usImagePostScan2D<Type>::getHeightResolution() const
{
  return m_heightResolution;
}

/**
* Getter for pixel width resolution.
* @return Width of a pixel (in meters).
*/
template<class Type>
double usImagePostScan2D<Type>::getWidthResolution() const
{
  return m_widthResolution;
}

/**
* Setter for pixel height resolution.
* @param heightResolution Height of a pixel (in meters).
*/
template<class Type>
void usImagePostScan2D<Type>::setHeightResolution(double heightResolution)
{
  m_heightResolution = heightResolution;
}

/**
* Setter for pixel width resolution.
* @param widthResolution Width of a pixel (in meters).
*/
template<class Type>
void usImagePostScan2D<Type>::setWidthResolution(double widthResolution)
{
  m_widthResolution = widthResolution;
}

#endif // US_IMAGE_POSTSCAN_2D_H
