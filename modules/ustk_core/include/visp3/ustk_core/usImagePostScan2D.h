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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImagePostScan2D.h
 * @brief 2D post-scan ultrasound image.
 */

#ifndef US_IMAGE_POSTSCAN_2D_H
#define US_IMAGE_POSTSCAN_2D_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePostScanSettings.h>

/**
 @class usImagePostScan2D
 @brief 2D post-scan ultrasound image.
 @ingroup module_ustk_core

 This class represents a 2D ultrasound post-scan frame.

   <h3>Example</h3>
  The following example shows how to build a 2D post-scan ultrasound image from a vpImage, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImagePostScan2D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      double probeRadius = 0.045;
      double scanLinePitch = 0.0012;
      bool isTransducerConvex = true;
      double heightResolution = 0.002;
      double widthResolution = 0.004;
      usImagePostScanSettings   imageSettings(probeRadius, scanLinePitch, isTransducerConvex, heightResolution, widthResolution);
      vpImage<unsigned char> I(AN, LN);
      usImagePostScan2D<unsigned char> postScan2d;
      postScan2d.setData(I);
      postScan2d.setImageSettings(imageSettings);
    }
  \endcode
 */
template<class Type>
class usImagePostScan2D : public vpImage<Type>, public usImagePostScanSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(unsigned int width, unsigned int height, double probeRadius=0.0, double scanLinePitch=0.0,
   bool isTransducerConvex=false, double height_resolution=0.0, double width_resolution=0.0);
  usImagePostScan2D(const usImagePostScan2D<Type> &other);
  usImagePostScan2D(const vpImage<Type> &other);
  usImagePostScan2D(const usImagePostScanSettings &other);
  usImagePostScan2D(const vpImage<Type> &otherImage, const usImagePostScanSettings &otherSettings);
  ~usImagePostScan2D();

  unsigned int getDimX() const;
  unsigned int getDimY() const;

  usImagePostScan2D<Type> & operator =(const usImagePostScan2D<Type> &other);
  friend VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usTransducerSettings &other);
  bool operator ==(const usImagePostScan2D<Type> &other);

  void setData(const vpImage<Type> &image);
};

/**
* Basic constructor, all parameters set to default values
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D() : vpImage<Type>(), usImagePostScanSettings()
{

}

/**
* Full constructor, all parameters settables.
* @param width number of pixels along x axis.
* @param height number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe
* used to acquire the RF image. Angle if \e isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param height_resolution Height resolution of the image.
* @param width_resolution Width resolution of the image.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(unsigned int width, unsigned int height,
double probeRadius, double scanLinePitch, bool isTransducerConvex,
double height_resolution, double width_resolution)
  : vpImage<Type>(width, height,0),
  usImagePostScanSettings(probeRadius, scanLinePitch, isTransducerConvex,
  height_resolution, width_resolution)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const usImagePostScan2D<Type> &other)
: vpImage<Type>(other), usImagePostScanSettings(other)
{

}


/**
* Constructor from vpImage
* @param other vpImage<unsigned char> to copy
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const vpImage<Type> &other) : vpImage<Type>(other)
{

}

/**
* Constructor from usImagePostScanSettings.
* @param other usImagePostScanSettings to copy
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const usImagePostScanSettings &other) : usImagePostScanSettings(other)
{

}

/**
* Constructor from vpImage and usImagePostScanSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usImagePostScanSettings to copy
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const vpImage<Type> &otherImage, const usImagePostScanSettings &otherSettings)
: vpImage<Type>(otherImage), usImagePostScanSettings(otherSettings)
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

  //from usImagePostScanSettings
  usImagePostScanSettings::operator=(other);

  return *this;
}

/**
* Comparaison operator.
*/
template<class Type>
bool usImagePostScan2D<Type>::operator ==(const usImagePostScan2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) &&
         usImagePostScanSettings::operator ==(other));
}

/**
* Operator to print image informations on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePostScan2D<Type> &other)
{
  return out << static_cast<const usImagePostScanSettings &>(other) <<
  "image width " << other.getWidth() << std::endl <<
  "image height : " << other.getHeight() << std::endl;
}

/**
* Returns the size of the image along the x axis (in pixels).
*/
template<class Type>
unsigned int usImagePostScan2D<Type>::getDimX() const {
  return vpImage<Type>::getWidth();
}

/**
* Returns the size of the image along the y axis (in pixels).
*/
template<class Type>
unsigned int usImagePostScan2D<Type>::getDimY() const {
  return vpImage<Type>::getHeight();
}

/**
* Setter for image.
* @param image Settings you want to copy.
*/
template<class Type>
void usImagePostScan2D<Type>::setData(const vpImage<Type> &image)
{
  vpImage<Type>::operator=(image);
}
#endif // US_IMAGE_POSTSCAN_2D_H
