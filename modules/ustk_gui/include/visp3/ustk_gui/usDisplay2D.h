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
 * @file usDisplay2D.h
 * @brief Class to display a ultrasound image at screen, and interact with it.
 */

#ifndef US_DISPLAY_2D_H
#define US_DISPLAY_2D_H

#include <string>

#include <visp3/core/vpDisplay.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>

#include <visp3/ustk_core/usPixelMeterConversion.h>
#include <visp3/ustk_core/usMeterPixelConversion.h>


/**
 * @class usDisplay2D
 * @brief Class to display a 2d ultrasound image at screen
 * @ingroup module_ustk_gui
 */
template<class Type>
class usDisplay2D
{
public:
  usDisplay2D();
  virtual ~usDisplay2D();

  void display();

  void setImage(usImagePostScan2D<unsigned char> imageToDisplay);

  void waitClick();


private:
  void includeImageInBackGround();
  vpDisplay * m_display;
  vpImage<unsigned char> m_backgroundImage;
  Type* m_usImage;
  vpImagePoint m_centerPoint; //position (in usImage coordinates system) of pixel (u0,v0) corresponding to conversion in pixel of meter position (x0,y0) = (0,0).
  vpImagePoint m_centerPointInBackground; //center point in background image coordinate system


  bool m_displayCenter; //true by default, but possible to hide the centerPoint of the usImage.
};


/**
* Default constructor.
*/
template<class Type>
usDisplay2D<Type>::usDisplay2D() : m_display(), m_backgroundImage(), m_usImage(),
  m_centerPoint(), m_centerPointInBackground(), m_displayCenter(true)
{
  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  m_display = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  m_display = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  m_display = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  m_display = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  m_display = new vpDisplayOpenCV;
#endif
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
  *m_usImage = imageToDisplay;
  //get center point
  double U0, V0;
  usMeterPixelConversion::convert(imageToDisplay,0,0,U0,V0);
  m_centerPoint.set_uv(U0,V0);
}

/**
* Displays the image.
*/
template<class Type>
void usDisplay2D<Type>::display()
{
  includeImageInBackGround();
  //init display
#ifdef VISP_HAVE_DISPLAY
  m_display->init(m_backgroundImage);
#endif

  // Specify the window location
  vpDisplay::setWindowPosition(m_backgroundImage, 400, 100);
  // Set the display window title
  vpDisplay::setTitle(m_backgroundImage, "2D ultrasound viewer");

  vpDisplay::display(m_backgroundImage);

  //display arrow of 1cm
  vpImagePoint endXArrow = vpImagePoint(m_centerPointInBackground.get_v(),m_centerPointInBackground.get_u() + m_usImage->getWidthResolution()*100);
  vpImagePoint endYArrow = vpImagePoint(m_centerPointInBackground.get_v() + m_usImage->getHeightResolution()*100,m_centerPointInBackground.get_u());
  vpDisplay::displayArrow 	(m_backgroundImage,m_centerPointInBackground, endXArrow, vpColor::red); //x in red
  vpDisplay::displayArrow 	(m_backgroundImage,m_centerPointInBackground, endYArrow, vpColor::green); //y in green
}

/**
* Include the image in background, with resizing option (if including center point).
*/
template<class Type>
void usDisplay2D<Type>::includeImageInBackGround()
{
  if(m_displayCenter) {
    m_backgroundImage.resize(m_usImage->getHeight() + std::abs(m_centerPoint.get_v()) + 10,
                             m_usImage->getWidth(), 0); // black background and 10px offset to not let the arrows at the border of the image
    m_backgroundImage.insert(*m_usImage,vpImagePoint(std::abs(m_centerPoint.get_v()),0));

    m_centerPointInBackground.set_uv(m_centerPoint.get_u(),10);
  }
  else
    m_backgroundImage.insert(*m_usImage,vpImagePoint(0,0));
}

/**
* Wait click to close display.
*/
template<class Type>
void usDisplay2D<Type>::waitClick()
{
  vpDisplay::getClick(m_backgroundImage);
}





#endif //US_DISPLAY_2D_H
