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
  usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN,
                   double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                   bool isImageConvex, bool isMotorConvex);
  //usImagePreScan3D copy constructor
  usImagePreScan3D(const usImagePreScan3D &other);
  //usImage3D copy constructor
  usImagePreScan3D(const usImage3D<T> &other);
  //usImageSettings3D copy constructor
  usImagePreScan3D(const usImageSettings3D &other);
  //copy constructor from usImage3D and usImageSettings3D
  usImagePreScan3D(const usImage3D<T> &other, const usImageSettings3D &otherSettings);
  //destructor
  ~usImagePreScan3D();

  //copying from usImage3D
  void copyFrom(const usImage3D<T> &I);

  double getAxialResolution() const;
  unsigned int getAN() const;
  unsigned int getFN() const;
  unsigned int getLN() const;

  //assignement
  usImagePreScan3D<T>& operator=(const usImagePreScan3D<T> &other);
  //comparison
  bool operator==(const usImagePreScan3D<T> &other);

  void setAxialResolution(double axialResolution);

private:
  double m_axialResolution;
};

/**
* Basic constructor, all settings set to default.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D() : usImage3D<T>(), usImageSettings3D()
{

}

/**
* Initializing image size constructor. For double image type.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN) : usImage3D<T>(LN, AN, FN), usImageSettings3D()
{

}

/**
* Initializing constructor for image size and probe settings. For double image type.
* @param AN A-samples in a line (corresponds to image height in px).
* @param LN Number of lines (corresponds to image width in px).
* @param FN Number of Frames.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param isImageConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(unsigned int AN, unsigned int LN, unsigned int FN, double probeRadius, double motorRadius, double scanLinePitch, double framePitch,
                                      bool isImageConvex, bool isMotorConvex) :
  usImage3D<T>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, scanLinePitch, framePitch, isImageConvex, isMotorConvex)
{

}

/**
* Copy constructor. For double image type.
* @param other usImagePreScan3D image you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImagePreScan3D &other) :
  usImage3D<T>(other), usImageSettings3D(other)
{

}

/**
* Copy constructor. For double image type.
* @param other usImage3D<double> image you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImage3D<T> &other) : usImage3D<T>(other)
{

}

/**
* Copy constructor.
* @param other usImageSettings3D you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImageSettings3D &other) : usImageSettings3D(other)
{

}

/**
* Copy constructor. For double image type.
* @param other usImageSettings3D you want to copy.
*/
template<class T>
usImagePreScan3D<T>::usImagePreScan3D(const usImage3D<T> &other, const usImageSettings3D &otherSettings) :
  usImage3D<T>(other), usImageSettings3D(otherSettings)
{

}

/**
* Destructor.
*/
template<class T>
usImagePreScan3D<T>::~usImagePreScan3D() {}

/**
* Copy operator.
*/
template<class T>
usImagePreScan3D<T>& usImagePreScan3D<T>::operator=(const usImagePreScan3D<T> &other)
{
  //from vpImage
  usImage3D<T>::operator=(other);

  //from usImageSettings
  usImageSettings3D::operator=(other);

  //from this class
  m_axialResolution = other.getAxialResolution();
}

/**
* Comparaison operator.
*/
template<class T>
bool usImagePreScan3D<T>::operator==(const usImagePreScan3D<T> &other)
{
  return(usImage3D<T>::operator== (other) &&
         usImageSettings3D::operator ==(other) &&
         m_axialResolution == other.getAxialResolution());
}

/**
* Copy from usImage3D. From double image type.
* @param I usImage3D<double> to copy.
*/
template<class T>
void usImagePreScan3D<T>::copyFrom(const usImage3D<T> &I)
{
  //resize(I.getHeight(), I.getWidth());B
  //memcpy(bitmap, I.bitmap, I.getSize() * sizeof(double));
}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
template<class T>
unsigned int usImagePreScan3D<T>::getAN() const { return usImage3D<T>::getHeight(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
template<class T>
unsigned int usImagePreScan3D<T>::getLN() const { return usImage3D<T>::getWidth(); }

/**
* Get the number of frames.
* @return FN number of frames.
*/
template<class T>
unsigned int usImagePreScan3D<T>::getFN() const { return usImage3D<T>::getDepth(); }

/**
* Getter for the axial resolution
* @return The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
double usImagePreScan3D<T>::getAxialResolution() const { return m_axialResolution; }

/**
* Setter for the axial resolution
* @param axialResolution The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
void usImagePreScan3D<T>::setAxialResolution(double axialResolution)
{
  m_axialResolution = axialResolution;
}

#endif // US_IMAGE_PRESCAN_3D_H
