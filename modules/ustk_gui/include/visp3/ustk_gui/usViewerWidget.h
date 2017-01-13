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
 * @file usViewerWidget.h
 * @brief View used to render a vtk scene in a QWidget (based on QVTKWidget)
 */


#ifndef __usViewerWidget_h_
#define __usViewerWidget_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#  include <QApplication>
#  include <QtGui/QMainWindow>
#  include <QtGui/QGridLayout>
#  include <QtGui/QPushButton>
#elif defined USTK_HAVE_VTK_QT5
#  include <QApplication>
#  include <QtWidgets/QMainWindow>
#  include <QtWidgets/QGridLayout>
#  include <QtWidgets/QPushButton>
#endif

#include <QVTKWidget.h>

/**
 * @class usViewerWidget
 * @brief View used to render a vtk scene in a QWidget (based on QVTKWidget)
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usViewerWidget : public QVTKWidget
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usViewerWidget(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  ~usViewerWidget() {}

  void paintEvent( QPaintEvent* event );
};
#endif
#endif // US_VIEWER_WIDGET
