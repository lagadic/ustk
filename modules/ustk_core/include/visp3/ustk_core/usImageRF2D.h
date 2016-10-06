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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImageRF2D.h
 * @brief 2D RF ultrasound image.
 */

#ifndef US_IMAGE_RF_2D_H
#define US_IMAGE_RF_2D_H

#include <cstring>

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/*!
 @class usImageRF2D
 @brief 2D RF ultrasound image.

 This class represents a 2D ultrasound RF image. This image is nothing more than a vpImage that
  contains RF data and additional settings that give information about the acquisition process.

  <h3>Example</h3>
  The following example shows how to build a RF2D ultrasound image from a vpImage, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImageRF2D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      double probeRadius = 0.007;
      double scanLinePitch = 0.0006;
      bool isTransducerConvex = true;
      double axialResolution = 0.002;
      usImagePreScanSettings imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      vpImage<unsigned char> I(AN, LN);
      usImageRF2D<unsigned char> rf2d;
      rf2d.setData(I);
      rf2d.setImageSettings(imageSettings);
    }
  \endcode
 */
template<class T>
class usImageRF2D : public vpImage<T>, public usImagePreScanSettings {
public:
  
  usImageRF2D();
  usImageRF2D(unsigned int a_number, unsigned int line_number, double probeRadius=0, double scanLinePitch=0, bool isTransducerConvex=true, double axialResolution=0);
  usImageRF2D(const usImageRF2D &other);
  ~usImageRF2D();

  unsigned int getANumber() const;
  unsigned int getLineNumber() const;

  usImageRF2D<T>& operator=(const usImageRF2D<T> &other);
  bool operator==(const usImageRF2D<T> &other);

  void setData(const vpImage<T> &image);
};


/**
* unsigned char
* Constructor.
*/
template<class T>
usImageRF2D<T>::usImageRF2D() : vpImage<T>(), usImagePreScanSettings()
{

}

/**
* Initializing constructor.
* @param a_number number of A-samples in a line.
* @param line_number number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe used
* to acquire the RF image. Angle if isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param axialResolution The distance (in meters) between 2 successive pixels acquired along a scanline.
*/
template<class T>
usImageRF2D<T>::usImageRF2D(unsigned int a_number, unsigned int line_number, double probeRadius, double scanLinePitch, bool isTransducerConvex, double axialResolution)
  : vpImage<T>(a_number, line_number), usImagePreScanSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution)
{

}

/**
* Copy constructor.
* @param other usImageRF2D to copy
*/
template<class T>
usImageRF2D<T>::usImageRF2D(const usImageRF2D& other)
  : vpImage<T>(other), usImagePreScanSettings(other)
{

}

/**
* Destructor.
*/
template<class T>
usImageRF2D<T>::~usImageRF2D()
{

}

/**
* Copy operator.
*/
template<class T>
usImageRF2D<T>& usImageRF2D<T>::operator=(const usImageRF2D<T> &other)
{
  //from vpImage
  vpImage<T>::operator=(other);

  //from usImagePreScanSettings
  usImagePreScanSettings::operator=(other);
}

/**
* Comparaison operator.
*/
template<class T>
bool usImageRF2D<T>::operator==(const usImageRF2D<T> &other)
{
  return(vpImage<T>::operator== (other) &&
         usImagePreScanSettings::operator ==(other));
}

/**
* Operator to print image informations on a stream.
*/
template<class T>
std::ostream& operator<<(std::ostream& out, const usImageRF2D<T> &other)
{
  return out << static_cast<const usImagePreScanSettings &>(other) <<
    "number of A-samples in a scanline : " << other.getANumber() << std::endl <<
    "number of scanlines : " << other.getLineNumber() << std::endl;
}

/**
* Get the number of A-samples in a line.
* @return a_number of A-samples in a line.
*/
template<class T>
unsigned int usImageRF2D<T>::getANumber() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @return line_number number of lines.
*/
template<class T>
unsigned int usImageRF2D<T>::getLineNumber() const { return vpImage<T>::getWidth(); }

/**
* Setter for the image data.
* @param image The image to set.
*/
template<class T>
void usImageRF2D<T>::setData(const vpImage<T> &image)
{
  vpImage<T>::operator=(image);
}

#endif // US_IMAGE_RF_2D_H
