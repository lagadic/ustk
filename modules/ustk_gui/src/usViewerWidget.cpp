/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#ifdef USTK_HAVE_VTK_QT

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>

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

  QVTKWidget::paintEvent( event );
/*  QPainter painter ( this );
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
#endif
