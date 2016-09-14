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
* @file usImagePreScan3D.h
* @brief 3D prescan ultrasound image.
*/

#ifndef US_IMAGE_PRESCAN_3D_H
#define US_IMAGE_PRESCAN_3D_H

#include <visp3/ustk_core/usImage3D.h>

#include <visp3/ustk_core/usImageSettings3D.h>

/**
* @class usImagePreScan3D
* @brief 3D prescan ultrasound image.
*
* This class represents a 3D ultrasound prescan frame.
*/
template<class T>
class usImagePreScan3D : public usImage3D<T>, public usImageSettings3D {
public:
  //default constructors
  usImagePreScan3D();

  //image size initialisation constructors
  usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN);

  //All parameters initialisation constructors
  usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN, float probeRadius, float motorRadius, float lineAngle, float frameAngle,
    float resolution, float BSampleFreq, float probeElementPitch);

  //usImagePreScan3D copy constructor
  usImagePreScan3D(const usImagePreScan3D &other);

  //usImage3D copy constructor
  usImagePreScan3D(const usImage3D<T> &other);

  //usImageSettings3D copy constructor
  usImagePreScan3D(const usImageSettings3D &other);

  //copy constructor from usImage3D and usImageSettings3D
  usImagePreScan3D(const usImage3D<T> &other, usImageSettings3D &otherSettings);

  //destructor
  ~usImagePreScan3D();

  //copying from usImage3D
  void copyFrom(const usImage3D<T> &I);

  unsigned int getAN() const;

  unsigned int getLN() const;

  unsigned int getFN() const;
};

#endif // US_IMAGE_PRESCAN_3D_H


/**
* Basic constructor, all settings set to default. For unsigned char data.
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D() : usImage3D<unsigned char>(), usImageSettings3D()
{

}

/**
* Basic constructor, all settings set to default. For double data.
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D() : usImage3D<double>(), usImageSettings3D()
{

}

/**
* Initializing image size constructor. For double image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN) : usImage3D<double>(LN, AN), usImageSettings3D()
{

}

/**
* Initializing image size constructor. For unsigned char image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN) : usImage3D<unsigned char>(LN, AN), usImageSettings3D()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements in the ultrasound probe.
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN, float probeRadius, float lineAngle,
  float resolution, float BSampleFreq, float probeElementPitch) :
  usImage3D<double>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, lineAngle, frameAngle, resolution, BSampleFreq, probeElementPitch)
{

}

/**
* Initializing constructor for image size and probe settings. For unsigned char image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
* @param[in] probeRadius Distance between the center point of the probe and the first pixel arc acquired, in meters (m).
* @param[in] lineAngle Radius between 2 successives acquisiton lines in the probe, in radians (rad).
* @param[in] resolution Size of a pixel (we use square pixels), in meters(m) for postscan. For prescan image (not managed yet) : line angle (in radians) and axial resolution (meters).
* @param[in] BSampleFreq Sampling frequency used for B-Mode.
* @param[in] probeElementPitch Physic parameter of the probe : distance between 2 sucessive piezoelectric elements in the ultrasound probe.
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN, float probeRadius, float motorRadius, float lineAngle, float frameAngle,
  float resolution, float BSampleFreq, float probeElementPitch) :
  usImage3D<unsigned char>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, lineAngle, frameAngle, resolution, BSampleFreq, probeElementPitch)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other usImagePreScan3D image you want to copy.
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D(const usImagePreScan3D &other) :
  usImage3D<double>(other), usImageSettings3D(other)
{

}

/**
* Copy constructor. For unsigned char image type.
* @param[in] other usImagePreScan3D image you want to copy.
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D(const usImagePreScan3D &other) :
  usImage3D<unsigned char>(other), usImageSettings3D(other)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other usImage3D<double> image you want to copy.
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D(const usImage3D<double> &other) : usImage3D<double>(other)
{

}

/**
* Copy constructor. For unsigned char image type.
* @param[in] other usImage3D<double> image you want to copy.
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D(const usImage3D<unsigned char> &other) : usImage3D<unsigned char>(other)
{

}

/**
* Copy constructor.
* @param[in] other usImageSettings3D you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImageSettings3D &other) : usImageSettings3D(other)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other usImageSettings3D you want to copy.
*/
template<double>
usImagePreScan3D<double>::usImagePreScan3D(const usImage3D<double> &other, usImageSettings3D &otherSettings) :
  usImage3D<double>(other), usImageSettings3D(otherSettings)
{

}

/**
*Copy constructor.For unsigned char image type.
* @param[in] other usImageSettings3D you want to copy.
*/
template<unsigned char>
usImagePreScan3D<unsigned char>::usImagePreScan3D(const usImage3D<unsigned char> &other, usImageSettings3D &otherSettings) :
  usImage3D<unsigned char>(other), usImageSettings3D(otherSettings)
{

}
/**
* Destructor.
*/
template<class T>
usImagePreScan3D<T>::~usImagePreScan3D() {};

/**
* Copy from usImage3D. From double image type.
* @param[in] I usImage3D<double> to copy.
*/
template<double>
void usImagePreScan3D<double>::copyFrom(const usImage3D<double> &I)
{
  //resize(I.getHeight(), I.getWidth());
  //memcpy(bitmap, I.bitmap, I.getSize() * sizeof(double));
}

/**
* Copy from usImage3D.
* @param[in] I usImage3D<unsigned char> to copy.
*/
template<unsigned char>
void usImagePreScan3D<unsigned char>::copyFrom(const usImage3D<unsigned char> &I)
{
  //resize(I.getHeight(), I.getWidth());
  //memcpy(bitmap, I.bitmap, I.getSize() * sizeof(unsigned char));
}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
template<class T>
unsigned int usImagePreScan3D::getAN() const { return usImage3D<T>::getHeight(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
template<class T>
unsigned int usImagePreScan3D::getLN() const { return usImage3D<T>::getWidth(); }

/**
* Get the number of frames.
* @return FN number of frames.
*/
template<class T>
unsigned int usImagePreScan3D::getFN() const { return usImage3D<T>::getDepth(); }
