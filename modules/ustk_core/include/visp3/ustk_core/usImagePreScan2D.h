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
        usImagePreScan2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex);

        //usImagePreScan2D copy constructor
        usImagePreScan2D(const usImagePreScan2D &other);

        //vpImage copy constructors
        usImagePreScan2D(const vpImage<T> &other);

        //vpImage copy constructors
        usImagePreScan2D(const usImageSettings &other);

        //copy constructor from vpImage and usImageSettings
        usImagePreScan2D(const vpImage<T> &other, const usImageSettings &otherSettings);

        //destructor
        ~usImagePreScan2D();

        //copying from vpImage
        void copyFrom(const vpImage<T> &I);

        //No setters for AN and LN because vpImage doesn't have setters for height and width. Those parameters have to be passed in the constructor.
        unsigned int getAN() const;

        unsigned int getLN() const;

        double getAxialResolution() const;

        void setAxialResolution(double axialResolution);

private:
  double m_axialResolution;
};

/**
* Basic constructor, all settings set to default. For double data.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D() : vpImage<T>(), usImageSettings()
{

}

/**
* Initializing image size constructor. For double image type.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(unsigned int AN, unsigned int LN) : vpImage<T>(LN, AN), usImageSettings()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param AN number of A-samples in a line.
* @param LN number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isConvex Boolean to specify if the probe used was convex(true) or linear(false).
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex) :
    vpImage<T>(AN, LN), usImageSettings(probeRadius, scanLinePitch, isConvex)
{

}

/**
* Copy constructor. For double image type.
* @param other usImagePreScan2D image you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const usImagePreScan2D &other) :
    vpImage<T>(other), usImageSettings(other)
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
* @param other usImageSettings you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const usImageSettings &other) : usImageSettings(other)
{

}

/**
* Copy constructor. For double image type.
* @param other usImageSettings you want to copy.
*/
template<class T>
usImagePreScan2D<T>::usImagePreScan2D(const vpImage<T> &other, const usImageSettings &otherSettings) :
    vpImage<T>(other), usImageSettings(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePreScan2D<T>::~usImagePreScan2D() {}

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
* @return AN number of A-samples in a line.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getAN() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
template<class T>
unsigned int usImagePreScan2D<T>::getLN() const { return vpImage<T>::getWidth(); }

/**
* Getter for the axial resolution
* @return The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
double usImagePreScan2D<T>::getAxialResolution() const { return m_axialResolution; }

/**
* Setter for the axial resolution
* @param axialResolution The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
void usImagePreScan2D<T>::setAxialResolution(double axialResolution)
{
  m_axialResolution = axialResolution;
}

#endif // US_IMAGE_PRESCAN_2D_H
