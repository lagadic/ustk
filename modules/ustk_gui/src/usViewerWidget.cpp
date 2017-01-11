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
* @file usViewerWidget.cpp
* @brief Graphical main window containing 4 vtk views.
*/

#include <visp3/ustk_gui/usViewerWidget.h>
#include <QPainter>

#include <QPaintEngine>

/**
* Constructor.
*/
usViewerWidget::usViewerWidget(QWidget* parent, Qt::WindowFlags f) : QVTKWidget(parent,f)
{

}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPAintEvent.
*/
void usViewerWidget::paintEvent( QPaintEvent* event )
{
/*
  QVTKWidget::paintEvent( event );
  QPainter painter ( this );
  QPen pen(Qt::red, 10, Qt::SolidLine);
  painter.setPen(pen);
  painter.drawLine(10, 10, 20, 20);

  QPolygon p = QPolygon();
  QPoint points[3];
  points[0] = QPoint(0, 0);
  points[1] = QPoint(10, 0);
  points[2] = QPoint(0, 10);
  QVTKWidget::paintEngine()->drawPolygon(points,3,QPaintEngine::WindingMode);*/
  //painter.drawPolygon( p );
}
