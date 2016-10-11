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
template<class T>
class usImagePostScan2D : public vpImage<T>, public usImagePostScanSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(unsigned int a_nubmer, unsigned int line_number, double probeRadius=0.0, double scanLinePitch=0.0, bool isTransducerConvex=false, double height_resolution=0.0, double width_resolution=0.0);
  usImagePostScan2D(const usImagePostScan2D<T> &other);
  usImagePostScan2D(const vpImage<T> &other);
  usImagePostScan2D(const usImagePostScanSettings &other);
  usImagePostScan2D(const vpImage<T> &otherImage, const usImagePostScanSettings &otherSettings);
  ~usImagePostScan2D();

  usImagePostScan2D<T> & operator =(const usImagePostScan2D<T> &other);

  bool operator ==(const usImagePostScan2D<T> &other);

  unsigned int getANumber() const;
  unsigned int getLineNumber() const;

  void setData(const vpImage<T> &image);
};

/**
* Basic constructor, all parameters set to default values
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D() : vpImage<T>(), usImagePostScanSettings()
{

}

/**
* Full constructor, all parameters settables.
* @param a_nubmer number of A-samples in a line.
* @param line_number number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe
* used to acquire the RF image. Angle if \e isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param height_resolution Height resolution of the image.
* @param width_resolution Width resolution of the image.
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(unsigned int a_nubmer, unsigned int line_number, double probeRadius, double scanLinePitch, bool isTransducerConvex, double height_resolution, double width_resolution)
  : vpImage<T>(a_nubmer, line_number,0), usImagePostScanSettings(probeRadius, scanLinePitch, isTransducerConvex, height_resolution, width_resolution)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImagePostScan2D<T> &other) : vpImage<T>(other), usImagePostScanSettings(other)
{

}


/**
* Constructor from vpImage
* @param other vpImage<unsigned char> to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const vpImage<T> &other) : vpImage<T>(other)
{

}

/**
* Constructor from usImagePostScanSettings.
* @param other usImagePostScanSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const usImagePostScanSettings &other) : usImagePostScanSettings(other)
{

}

/**
* Constructor from vpImage and usImagePostScanSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usImagePostScanSettings to copy
*/
template<class T>
usImagePostScan2D<T>::usImagePostScan2D(const vpImage<T> &otherImage, const usImagePostScanSettings &otherSettings) : vpImage<T>(otherImage), usImagePostScanSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePostScan2D<T>::~usImagePostScan2D() {}

/**
* Assignement operator.
*/
template<class T>
usImagePostScan2D<T> & usImagePostScan2D<T>::operator =(const usImagePostScan2D<T> &other)
{
  //from vpImage
  vpImage<T>::operator=(other);

  //from usImagePostScanSettings
  usImagePostScanSettings::operator=(other);

  return *this;
}

/**
* Comparaison operator.
*/
template<class T>
bool usImagePostScan2D<T>::operator ==(const usImagePostScan2D<T> &other)
{
  return(vpImage<T>::operator== (other) &&
         usImagePostScanSettings::operator ==(other));
}

/**
* Operator to print image informations on a stream.
*/
template<class T>
std::ostream& operator<<(std::ostream& out, const usImagePostScan2D<T> &other)
{
  return out << static_cast<const usImagePostScanSettings &>(other) <<
  "number of A-samples in a scanline : " << other.getANumber() << std::endl <<
  "number of scanlines : " << other.getLineNumber() << std::endl;
}

/**
* Get the number of A-samples in a line.
* @return a_number Number of A-samples in a line.
*/
template<class T>
unsigned int usImagePostScan2D<T>::getANumber() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @return line_number number of lines.
*/
template<class T>
unsigned int usImagePostScan2D<T>::getLineNumber() const { return vpImage<T>::getWidth(); }

/**
* Setter for image.
* @param image Settings you want to copy.
*/
template<class T>
void usImagePostScan2D<T>::setData(const vpImage<T> &image)
{
  vpImage<T>::operator=(image);
}
#endif // US_IMAGE_POSTSCAN_2D_H
