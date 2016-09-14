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
 * @brief 2D prescan ultrasound image.
 */

#ifndef US_IMAGE_PRESCAN_2D_H
#define US_IMAGE_PRESCAN_2D_H

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImagePreScan2D
 * @brief 2D prescan ultrasound image.
 *
 * This class represents a 2D ultrasound prescan frame.
 */
template<class T>
class usImagePreScan2D : public vpImage<T>, public usImageSettings {
    public:
        //default constructors
        usImagePreScan2D();

        //image size initialisation constructors
        usImagePreScan2D(unsigned int AN, unsigned int LN);

        //All parameters initialisation constructors
        usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
		      float resolution, float BSampleFreq, float probeElementPitch);

        //usImagePreScan2D copy constructor
        usImagePreScan2D(const usImagePreScan2D &other);

        //vpImage copy constructors
        usImagePreScan2D(const vpImage<T> &other);

        //vpImage copy constructors
        usImagePreScan2D(const usImageSettings &other);

        //copy constructor from vpImage and usImageSettings
        usImagePreScan2D(const vpImage<T> &other, usImageSettings &otherSettings);

        //destructor
        ~usImagePreScan2D();

        //copying from vpImage
        void copyFrom(const vpImage<T> &I);

        //No setters for AN and LN because vpImage doesn't have setters for height and width. Those parameters have to be passed in the constructor.
        unsigned int getAN() const;

        unsigned int getLN() const;

};

#endif // US_IMAGE_PRESCAN_2D_H


/**
* Basic constructor, all settings set to default. For unsigned char data.
*/
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D() : vpImage<unsigned char>(), usImageSettings()
{

}

/**
* Basic constructor, all settings set to default. For double data.
*/
template<>
usImagePreScan2D<double>::usImagePreScan2D() : vpImage<double>(), usImageSettings()
{

}

/**
* Initializing image size constructor. For double image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
*/
template<>
usImagePreScan2D<double>::usImagePreScan2D(unsigned int AN, unsigned int LN) : vpImage<double>(LN, AN), usImageSettings()
{

}

/**
* Initializing image size constructor. For unsigned char image type.
* @param[in] AN A-samples in a line (corresponds to image height in px).
* @param[in] LN Number of lines (corresponds to image width in px).
*/
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D(unsigned int AN, unsigned int LN) : vpImage<unsigned char>(LN, AN), usImageSettings()
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
template<>
usImagePreScan2D<double>::usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
    float resolution, float BSampleFreq, float probeElementPitch) :
    vpImage<double>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
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
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
    float resolution, float BSampleFreq, float probeElementPitch) :
    vpImage<unsigned char>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other usImagePreScan2D image you want to copy.
*/
template<>
usImagePreScan2D<double>::usImagePreScan2D(const usImagePreScan2D &other) :
    vpImage<double>(other), usImageSettings(other)
{

}

/**
* Copy constructor. For unsigned char image type.
* @param[in] other usImagePreScan2D image you want to copy.
*/
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D(const usImagePreScan2D &other) :
    vpImage<unsigned char>(other), usImageSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other vpImage<double> image you want to copy.
*/
template<>
usImagePreScan2D<double>::usImagePreScan2D(const vpImage<double> &other) : vpImage<double>(other)
{

}

/**
* Copy constructor. For unsigned char image type.
* @param[in] other vpImage<double> image you want to copy.
*/
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D(const vpImage<unsigned char> &other) : vpImage<unsigned char>(other)
{

}

/**
* Copy constructor.
* @param[in] other usImageSettings you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const usImageSettings &other) : usImageSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param[in] other usImageSettings you want to copy.
*/
template<>
usImagePreScan2D<double>::usImagePreScan2D(const vpImage<double> &other, usImageSettings &otherSettings) :
    vpImage<double>(other), usImageSettings(otherSettings)
{

}

/**
*Copy constructor.For unsigned char image type.
* @param[in] other usImageSettings you want to copy.
*/
template<>
usImagePreScan2D<unsigned char>::usImagePreScan2D(const vpImage<unsigned char> &other, usImageSettings &otherSettings) :
    vpImage<unsigned char>(other), usImageSettings(otherSettings)
{

}
/**
* Destructor.
*/
template<class T>
usImagePreScan2D<T>::~usImagePreScan2D() {};

/**
* Copy from vpImage. From double image type.
* @param[in] I vpImage<double> to copy.
*/
template<>
void usImagePreScan2D<double>::copyFrom(const vpImage<double> &I)
{
    resize(I.getHeight(), I.getWidth());
    memcpy(bitmap, I.bitmap, I.getSize() * sizeof(double));
}

/**
* Copy from vpImage.
* @param[in] I vpImage<unsigned char> to copy.
*/
template<>
void usImagePreScan2D<unsigned char>::copyFrom(const vpImage<unsigned char> &I)
{
    resize(I.getHeight(), I.getWidth());
    memcpy(bitmap, I.bitmap, I.getSize() * sizeof(unsigned char));
}

/**
* Get the number of A-samples in a line.
* @param[out] AN number of A-samples in a line.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getAN() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @param[out] LN number of lines.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getLN() const { return vpImage<T>::getWidth(); }
