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
 * @file us3DSceneSlicing.h
 * @brief Graphical main window
 */


#ifndef __us3DSceneSlicing_h_
#define __us3DSceneSlicing_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

// USTK includes

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/us3DSceneWidget.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPlane.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>
#include <vtkImageMapper3D.h>


// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#  include <QtGui/QApplication>
#  include <QtGui/QMainWindow>
#  include <QtGui/QGridLayout>
#  include <QtGui/QPushButton>
#  include <QtGui/QSlider>
#elif defined(USTK_HAVE_VTK_QT5)
#  include <QtWidgets/QApplication>
#  include <QtWidgets/QMainWindow>
#  include <QtWidgets/QGridLayout>
#  include <QtWidgets/QPushButton>
#  include <QtWidgets/QSlider>
#endif

/**
 * @class us3DSceneSlicing
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT us3DSceneSlicing : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  us3DSceneSlicing(std::string imageFileName);
  ~us3DSceneSlicing() {}

  void resizeEvent(QResizeEvent* event);

public slots:
  void updateX(int x);
  void updateY(int y);
  void updateZ(int z);

  void updateRotX(int rx);
  void updateRotY(int ry);
  void updateRotZ(int rz);

  virtual void ResetViews();
  virtual void Render();
  virtual void slotExit();

private:
    void setupUi();

    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    us3DSceneWidget *view;

    //plane 1 modifiers
    //translations
    QSlider *sliderXplane1;
    QSlider *sliderYplane1;
    QSlider *sliderZplane1;
    //rotations
    QSlider *rotXplane1;
    QSlider *rotYplane1;
    QSlider *rotZplane1;

    //VTK planes
    vtkPlane *plane1;
    vtkPlane *plane2;
    vtkPlane *plane3;

    //transformation
    vtkMatrix4x4 *vtkMatrix1;
    vtkMatrix4x4 *vtkMatrix2;
    vtkMatrix4x4 *vtkMatrix3;

    //image
    usImagePostScan3D<unsigned char> postScanImage;
    vtkSmartPointer<vtkImageData> vtkImage;
};
#endif
#endif // __us3DSceneSlicing_h_
