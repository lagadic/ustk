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

#include <visp3/ustk_core/usTransducerSettings.h>

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
      unsigned int dimX = 200;
      unsigned int dimY = 200;
      double probeRadius = 0.045;
      double scanLinePitch = 0.0012;
      bool isTransducerConvex = true;
      double heightResolution = 0.002;
      double widthResolution = 0.004;
      usImagePostScanSettings imageSettings(probeRadius, scanLinePitch, isTransducerConvex, heightResolution, widthResolution);
      vpImage<unsigned char> I(dimY,dimX);
      usImagePostScan2D<unsigned char> postScan2d;
      postScan2d.setData(I);
      postScan2d.setImageSettings(imageSettings);
    }
  \endcode
 */
template<class Type>
class usImagePostScan2D : public vpImage<Type>, public usTransducerSettings {
public:
  usImagePostScan2D();
  usImagePostScan2D(unsigned int width, unsigned int height, double probeRadius=0.0, double scanLinePitch=0.0,
   bool isTransducerConvex=false, unsigned int scanLineNumber = 0,double widthResolution = 0.0, double heightResolution=0.0);
  usImagePostScan2D(const usImagePostScan2D<Type> &other);
  usImagePostScan2D(const vpImage<Type> &otherImage, usTransducerSettings otherSettings, unsigned int scanLineNumber = 0, double widthResolution = 0.0, double heightResolution = 0.0);
  virtual ~usImagePostScan2D();

  double getHeightResolution() const;
  unsigned int getScanLineNumber() const;
  double getWidthResolution() const;

  usImagePostScan2D<Type> & operator =(const usImagePostScan2D<Type> &other);
  bool operator ==(const usImagePostScan2D<Type> &other);

  void setData(const vpImage<Type> &image);
  void setHeightResolution(double heightResolution);
  void setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex, unsigned int scanLineNumber, double widthResolution, double heightResolution);
  void setScanLineNumber(unsigned int scanLineNumber);
  void setWidthResolution(double widthResolution);

private:
  double m_widthResolution;
  double m_heightResolution;
  unsigned int m_scanLineNumber;
};

/**
* Basic constructor, all parameters set to default values
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D() : vpImage<Type>(), usTransducerSettings(), m_widthResolution(0.0), m_heightResolution(0.0)
{

}

/**
* Full constructor, all parameters settables.
* @param width number of pixels along x axis.
* @param height number of pixels along y axis.
* @param probeRadius Radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe
* used to acquire the RF image. Angle if \e isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param heightResolution Height resolution of the image.
* @param widthResolution Width resolution of the image.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(unsigned int width, unsigned int height,
double probeRadius, double scanLinePitch, bool isTransducerConvex, unsigned int scanLineNumber,
double widthResolution, double heightResolution)
  : vpImage<Type>(width, height,(Type)0),
  usTransducerSettings(probeRadius, scanLinePitch, isTransducerConvex),
  m_widthResolution(widthResolution), m_heightResolution(heightResolution), m_scanLineNumber(scanLineNumber)
{

}

/**
* Copy constructor from other usImagePostScan2D
* @param other usImagePostScan2D to copy
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const usImagePostScan2D<Type> &other)
: vpImage<Type>(other), usTransducerSettings(other), 
  m_widthResolution(other.getWidthResolution()), m_heightResolution(other.getHeightResolution()),
  m_scanLineNumber(other.getScanLineNumber())
{

}

/**
* Constructor from vpImage and usTransducerSettings.
* @param otherImage vpImage<unsigned char> to copy
* @param otherSettings usTransducerSettings to copy
* @param heightResolution Height (in meters) of a pixel.
* @param widthResolution Width (in meters) of a pixel.
*/
template<class Type>
usImagePostScan2D<Type>::usImagePostScan2D(const vpImage<Type> &otherImage, usTransducerSettings otherSettings, unsigned int scanLineNumber,
                                           double widthResolution, double heightResolution)
: vpImage<Type>(otherImage), usTransducerSettings(otherSettings), m_widthResolution(widthResolution), m_heightResolution(heightResolution),
  m_scanLineNumber(scanLineNumber)
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
  m_scanLineNumber = other.getScanLineNumber();
  m_heightResolution = other.getHeightResolution();
  m_widthResolution = other.getWidthResolution();

  return *this;
}

/**
* Comparaison operator.
*/
template<class Type>
bool usImagePostScan2D<Type>::operator ==(const usImagePostScan2D<Type> &other)
{
  return(vpImage<Type>::operator== (other) &&
      usTransducerSettings::operator ==(other) &&
      m_scanLineNumber == other.getScanLineNumber() &&
      m_heightResolution == other.getHeightResolution() &&
      m_widthResolution == other.getWidthResolution());
}

/**
* Operator to print image informations on a stream.
*/
template<class Type>
std::ostream& operator<<(std::ostream& out, const usImagePostScan2D<Type> &other)
{
  return out << static_cast<const usTransducerSettings &>(other) <<
                "image width : " << other.getWidth() << std::endl <<
                "image height : " << other.getHeight() << std::endl <<
                "scanline number : " << other.getScanLineNumber() << std::endl <<
                "image width resolution : " << other.getWidthResolution() <<
                "image height resolution : " << other.getHeightResolution() << std::endl;
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

/**
* Getter for height resolution.
* @return Height of a pixel (in meters).
*/
template<class Type>
double usImagePostScan2D<Type>::getHeightResolution() const
{
  return m_heightResolution;
}

/**
* Getter for width resolution.
* @return Width of a pixel (in meters).
*/
template<class Type>
double usImagePostScan2D<Type>::getWidthResolution() const
{
  return m_widthResolution;
}

/**
* Setter for height resolution.
* @param heightResolution Height of a pixel (in meters).
*/
template<class Type>
void usImagePostScan2D<Type>::setHeightResolution(double heightResolution)
{
  m_heightResolution = heightResolution;
}

/**
* Setter for width resolution.
* @param widthResolution Width of a pixel (in meters).
*/
template<class Type>
void usImagePostScan2D<Type>::setWidthResolution(double widthResolution)
{
  m_widthResolution = widthResolution;
}

/**
* Setter for all 2D post-scan settings.
* @param probeRadius Radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle (radians) or distance (meters) between 2 lines of the ultrasound probe
* used to acquire the RF image. Angle if \e isTransducerConvex is true, distance otherwise.
* @param isTransducerConvex Boolean to specify if the probe transducer is convex (true) or linear (false).
* @param widthResolution Width of a pixel (in meters).
* @param heightResolution Height of a pixel (in meters).
*/
template<class Type>

void usImagePostScan2D<Type>::setImageSettings(double probeRadius, double scanLinePitch, bool isTransducerConvex,
                                               unsigned int scanLineNumber, double widthResolution, double heightResolution)

{
  setTransducerConvexity(isTransducerConvex);
  setProbeRadius(probeRadius);
  setScanLinePitch(scanLinePitch);
  m_widthResolution = widthResolution;
  m_heightResolution = heightResolution;
  m_scanLineNumber = scanLineNumber;
}

/**
* Getter for scanLineNumber.
* @return Number of scanlines in the probe used.
*/
template<class Type>
unsigned int usImagePostScan2D<Type>::getScanLineNumber() const
{
  return m_scanLineNumber;
}

/**
* Setter for scanLineNumber.
* @param scanLineNumber Number of scanlines in the probe used.
*/
template<class Type>
void usImagePostScan2D<Type>::setScanLineNumber(unsigned int scanLineNumber)
{
  m_scanLineNumber =  scanLineNumber;
}
#endif // US_IMAGE_POSTSCAN_2D_H
