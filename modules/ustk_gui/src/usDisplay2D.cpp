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

#include <visp3/ustk_gui/usDisplay2D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

/**
* Default constructor.
*/
template<class Type>
usDisplay2D<Type>::usDisplay2D() : m_display(new vpDisplayOpenCV()), m_backgroundImage(vpImage<unsigned char>(100,100,0)), m_usImage(Type())
{
  m_display->init(m_backgroundImage);
  //vpDisplay::setWindowPosition(I, 400, 100);
  //vpDisplay::setTitle(I, "Ultrasound image");
}

/**
* Destructor.
*/
template<class Type>
usDisplay2D<Type>::~usDisplay2D()
{
  delete m_display;
}

/**
* Image setter.
*/
template<class Type>
void usDisplay2D<Type>::setImage(usImagePostScan2D<unsigned char> imageToDisplay)
{
  //vpDisplay::display();
}

