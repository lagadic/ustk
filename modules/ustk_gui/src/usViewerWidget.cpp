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
#include <vtkDistanceWidget.h>
#include <vtkImagePlaneWidget.h>
#include <vtkResliceImageViewer.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkSmartPointer.h>

#include <QPaintEngine>
#include <QPainter>

/**
* Constructor.
*/
#if USTK_HAVE_VTK_VERSION < 0x090000
usViewerWidget::usViewerWidget(QWidget *parent, Qt::WindowFlags f) : QVTKWidget(parent, f) {}
#else
usViewerWidget::usViewerWidget(QWidget *parent, Qt::WindowFlags f) : QVTKOpenGLStereoWidget(parent, f) {}
#endif

usViewerWidget::~usViewerWidget() {}

/**
* Qt paint event overload if needed to update Qt widget
* @param event QPAintEvent.
*/
#if USTK_HAVE_VTK_VERSION < 0x090000
void usViewerWidget::paintEvent(QPaintEvent *event) { QVTKWidget::paintEvent(event); }
#else
void usViewerWidget::paintEvent(QPaintEvent *event) { QVTKOpenGLStereoWidget::paintEvent(event); }
#endif
#endif
