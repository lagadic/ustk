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

/**
 * @file usImagePreScan2D.h
 * @brief 2D pre-scan ultrasound image.
 */

#ifndef US_IMAGE_PRESCAN_2D_H
#define US_IMAGE_PRESCAN_2D_H

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImagePreScanSettings.h>

/**
 @class usImagePreScan2D
 @brief 2D pre-scan ultrasound image.

  <h3>Example</h3>
  The following example shows how to build a 2D pre-scan ultrasound image from a usImage3D, and from acquisiton settings.

  \code
    #include <visp3/ustk_core/usImagePreScan2D.h>

    int main()
    {
      // Update settings
      unsigned int AN = 200;
      unsigned int LN = 200;
      double probeRadius = 0.06;
      double scanLinePitch = 0.04;
      bool isTransducerConvex = true;
      double axialResolution = 0.005;
      usImagePreScanSettings imageSettings(probeRadius, scanLinePitch, isTransducerConvex, axialResolution);
      vpImage<unsigned char> I(AN, LN);
      usImagePreScan2D<unsigned char> preScan2d;
      preScan2d.setData(I);
      preScan2d.setImageSettings(imageSettings);
    }
  \endcode

 This class represents a 2D ultrasound pre-scan frame.
 */
template<class T>
class usImagePreScan2D : public vpImage<T>, public usImagePreScanSettings {
public:
  //default constructors
  usImagePreScan2D();
  //All parameters initialisation constructors
  usImagePreScan2D(unsigned int a_number, unsigned int line_number, double probeRadius=0.0, double scanLinePitch=0.0, bool isTransducerConvex=false, double axial_resolution=0.0);
  //usImagePreScan2D copy constructor
  usImagePreScan2D(const usImagePreScan2D &other);
  //vpImage copy constructors
  usImagePreScan2D(const vpImage<T> &other);
  //vpImage copy constructors
  usImagePreScan2D(const usImagePreScanSettings &other);
  //copy constructor from vpImage and usImagePreScanSettings
  usImagePreScan2D(const vpImage<T> &other, const usImagePreScanSettings &otherSettings);

  //destructor
  ~usImagePreScan2D();

  //copying from vpImage
  void copyFrom(const vpImage<T> &I);

  //No setters for a_number and LN because vpImage doesn't have setters for height and width. Those parameters have to be passed in the constructor.
  unsigned int getANumber() const;
  unsigned int getLineNumber() const;

  //assignement
  usImagePreScan2D<T>& operator=(const usImagePreScan2D<T> &other);

  //comparison
  bool operator==(const usImagePreScan2D<T> &other);

  void setData(const vpImage<T> image);
  void setImageSettings(const usImagePreScanSettings &settings);
};

/**
* Basic constructor, all settings set to default. For double data.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D() : vpImage<T>(), usImagePreScanSettings()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param a_number number of A-samples in a line.
* @param line_number number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe used
* to acquire the RF image. Angle if \e isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param axial_resolution Image axial resolution.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(unsigned int a_number, unsigned int line_number, double probeRadius, double scanLinePitch, bool isTransducerConvex, double axial_resolution) :
  vpImage<T>(a_number, line_number), usImagePreScanSettings(probeRadius, scanLinePitch, isTransducerConvex, axial_resolution)
{

}

/**
* Copy constructor. For double image type.
* @param other usImagePreScan2D image you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const usImagePreScan2D &other) :
  vpImage<T>(other), usImagePreScanSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param other vpImage<double> image you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const vpImage<T> &other) : vpImage<T>(other)
{

}

/**
* Copy constructor.
* @param other usImagePreScanSettings you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const usImagePreScanSettings &other) : usImagePreScanSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param other vpImage you want to copy.
* @param otherSettings usImagePreScanSettings you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const vpImage<T> &other, const usImagePreScanSettings &otherSettings) :
  vpImage<T>(other), usImagePreScanSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePreScan2D<T>::~usImagePreScan2D() {}

/**
* Copy operator.
*/
template<class T>
usImagePreScan2D<T>& usImagePreScan2D<T>::operator=(const usImagePreScan2D<T> &other)
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
bool usImagePreScan2D<T>::operator==(const usImagePreScan2D<T> &other)
{
  return(vpImage<T>::operator== (other) &&
         usImagePreScanSettings::operator ==(other));
}

/**
* Copy from vpImage. From double image type.
* @param I vpImage<double> to copy.
*/
template<class T>
void usImagePreScan2D<T>::copyFrom(const vpImage<T> &I)
{
  resize(I.getHeight(), I.getWidth());
  memcpy(this->bitmap, I.bitmap, I.getSize() * sizeof(T));
}

/**
* Get the number of A-samples in a line.
* @return a_number Number of A-samples in a line.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getANumber() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @return line_number number of lines.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getLineNumber() const { return vpImage<T>::getWidth(); }

/**
* Setter for the image data.
* @param image The image to set.
*/
template<class T>
void usImagePreScan2D<T>::setData(const vpImage<T> image)
{
  vpImage<T>::operator=(image);
}

/**
* Setter for the image settings.
* @param settings The new image settings.
*/
template<class T>
void usImagePreScan2D<T>::setImageSettings(const usImagePreScanSettings &settings)
{
  usImagePreScanSettings::operator=(settings);
}
#endif // US_IMAGE_PRESCAN_2D_H
