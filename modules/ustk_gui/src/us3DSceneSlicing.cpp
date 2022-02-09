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
* @file us3DSceneSlicing.cpp
* @brief Graphical main window containing 4 vtk views.
*/

#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_gui/us3DSceneSlicing.h>
#include <visp3/ustk_gui/usVTKConverter.h>

#ifdef USTK_HAVE_VTK_QT

#include <vtkAbstractTransform.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkBoundedPlanePointPlacer.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation2D.h>
#include <vtkDistanceWidget.h>
#include <vtkHandleRepresentation.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageMapToWindowLevelColors.h>
#include <vtkImageSlabReslice.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkMetaImageReader.h>
#include <vtkPlane.h>
#include <vtkPlaneSource.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkResliceCursor.h>
#include <vtkResliceCursorActor.h>
#include <vtkResliceCursorLineRepresentation.h>
#include <vtkResliceCursorPolyDataAlgorithm.h>
#include <vtkResliceCursorThickLineRepresentation.h>
#include <vtkResliceCursorWidget.h>
#include <vtkResliceImageViewer.h>
#include <vtkResliceImageViewerMeasurements.h>

#include <vtkHomogeneousTransform.h>

#include <QDesktopWidget>
#include <QResizeEvent>

/**
* Constructor.
* @param imageFileName the mhd file to read.
*/
us3DSceneSlicing::us3DSceneSlicing(std::string imageFileName)
{
  this->setupUi();
  usImageIo::read(postScanImage, imageFileName);
  usVTKConverter::convert(postScanImage, vtkImage);

  int imageDims[3];
  double spacing[3];
  vtkImage->GetDimensions(imageDims);
  vtkImage->GetSpacing(spacing);

  plane1 = vtkPlane::New();
  plane1->SetOrigin(imageDims[0] * spacing[0] / 2, 0, 0);
  plane1->SetNormal(1, 0, 0);

  plane2 = vtkPlane::New();
  plane2->SetNormal(0, 1, 0);
  plane2->SetOrigin(0, imageDims[1] * spacing[1] / 2, 0);

  plane3 = vtkPlane::New();
  plane3->SetNormal(0, 0, 1);
  plane3->SetOrigin(0, 0, imageDims[2] * spacing[2] / 2);

  this->view->setImageData(vtkImage);
  this->view->setPlanes(plane1, plane2, plane3);
  this->view->init();

  sliderXplane1->setMaximum(imageDims[0] * spacing[0] * 1000);
  sliderXplane1->setSliderPosition(imageDims[0] * spacing[0] * 1000 / 2);
  sliderYplane1->setMaximum(imageDims[1] * spacing[1] * 1000);
  sliderYplane1->setSliderPosition(imageDims[1] * spacing[1] * 1000 / 2);
  sliderZplane1->setMaximum(imageDims[2] * spacing[2] * 1000);
  sliderZplane1->setSliderPosition(imageDims[2] * spacing[2] * 1000 / 2);

  // Set up action signals and slots
  connect(this->sliderXplane1, SIGNAL(valueChanged(int)), this, SLOT(updateX(int)));
  connect(this->sliderYplane1, SIGNAL(valueChanged(int)), this, SLOT(updateY(int)));
  connect(this->sliderZplane1, SIGNAL(valueChanged(int)), this, SLOT(updateZ(int)));

  connect(this->rotXplane1, SIGNAL(valueChanged(int)), this, SLOT(updateRotX(int)));
  connect(this->rotYplane1, SIGNAL(valueChanged(int)), this, SLOT(updateRotY(int)));
  connect(this->rotZplane1, SIGNAL(valueChanged(int)), this, SLOT(updateRotZ(int)));

  ResetViews();
}

/**
* Exit slot, to exit the QApplication.
*/
void us3DSceneSlicing::slotExit() { qApp->exit(); }

/**
* Reset views slot : reset the planes positions at the middle of the volume.
*/
void us3DSceneSlicing::ResetViews() { this->Render(); }

/**
* Render slot, to recompute all the views.
*/
void us3DSceneSlicing::Render()
{
#if USTK_HAVE_VTK_VERSION < 0x090000
  this->view->GetRenderWindow()->Render();
#else
  this->view->renderWindow()->Render();
#endif

  this->view->update();
}

/**
* Setup all the widgets in the window.
*/
void us3DSceneSlicing::setupUi()
{
  this->setMinimumSize(640, 480);
  QRect screenRect = QApplication::desktop()->screenGeometry();
  this->resize(screenRect.size());

  gridLayoutWidget = new QWidget(this);
  gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
  gridLayoutWidget->setGeometry(QRect(10, 10, screenRect.width() - 200, screenRect.height() - 40));
  gridLayout_2 = new QGridLayout(gridLayoutWidget);
  gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
  gridLayout_2->setContentsMargins(0, 0, 0, 0);
  view = new us3DSceneWidget(gridLayoutWidget);
  view->setObjectName(QString::fromUtf8("view"));

  gridLayout_2->addWidget(view, 0, 0, 1, 1);

  sliderXplane1 = new QSlider(this);
  sliderXplane1->setObjectName(QString::fromUtf8("sliderX"));
  sliderXplane1->setMinimum(0);
  sliderXplane1->setMaximum(100);
  sliderXplane1->setGeometry(QRect(screenRect.width() - 180, 30, 20, 200));

  sliderYplane1 = new QSlider(this);
  sliderYplane1->setObjectName(QString::fromUtf8("sliderY"));
  sliderYplane1->setMinimum(0);
  sliderYplane1->setMaximum(100);
  sliderYplane1->setGeometry(QRect(screenRect.width() - 140, 30, 20, 200));

  sliderZplane1 = new QSlider(this);
  sliderZplane1->setObjectName(QString::fromUtf8("sliderZ"));
  sliderZplane1->setMinimum(0);
  sliderZplane1->setMaximum(100);
  sliderZplane1->setGeometry(QRect(screenRect.width() - 100, 30, 20, 200));

  rotXplane1 = new QSlider(this);
  rotXplane1->setObjectName(QString::fromUtf8("rotX"));
  rotXplane1->setMinimum(0);
  rotXplane1->setMaximum(100);
  rotXplane1->setGeometry(QRect(screenRect.width() - 180, 250, 20, 200));

  rotYplane1 = new QSlider(this);
  rotYplane1->setObjectName(QString::fromUtf8("rotY"));
  rotYplane1->setMinimum(0);
  rotYplane1->setMaximum(100);
  rotYplane1->setGeometry(QRect(screenRect.width() - 140, 250, 20, 200));

  rotZplane1 = new QSlider(this);
  rotZplane1->setObjectName(QString::fromUtf8("rotZ"));
  rotZplane1->setMinimum(0);
  rotZplane1->setMaximum(100);
  rotZplane1->setGeometry(QRect(screenRect.width() - 100, 250, 20, 200));
}

/**
* Get the resize event of the window, to re-comute size and positions of all widgets/layouts.
*/
void us3DSceneSlicing::resizeEvent(QResizeEvent *event)
{
  // Min size : 640*480
  if (event->size().width() >= 640 && event->size().height() >= 480) {
    QMainWindow::resizeEvent(event);
    gridLayoutWidget->setGeometry(QRect(10, 10, event->size().width() - 220, event->size().height() - 20));
    sliderXplane1->setGeometry(QRect(event->size().width() - 180, 30, 20, 200));
    sliderYplane1->setGeometry(QRect(event->size().width() - 140, 30, 20, 200));
    sliderZplane1->setGeometry(QRect(event->size().width() - 100, 30, 20, 200));
    rotXplane1->setGeometry(QRect(event->size().width() - 180, 250, 20, 200));
    rotYplane1->setGeometry(QRect(event->size().width() - 140, 250, 20, 200));
    rotZplane1->setGeometry(QRect(event->size().width() - 100, 250, 20, 200));
  }
}

/**
* Updates X translation.
*/
void us3DSceneSlicing::updateX(int x)
{
  double origin[3];
  plane1->GetOrigin(origin);
  origin[0] = x / 1000.0;
  plane1->SetOrigin(origin);
  view->update();
}

/**
* Updates Y translation.
*/
void us3DSceneSlicing::updateY(int y)
{
  double origin[3];
  plane1->GetOrigin(origin);
  origin[1] = y / 1000.0;
  plane1->SetOrigin(origin);
  view->update();
}

/**
* Updates Z translation.
*/
void us3DSceneSlicing::updateZ(int z)
{
  double origin[3];
  plane1->GetOrigin(origin);
  origin[2] = z / 1000.0;
  plane1->SetOrigin(origin);
  view->update();
}

/**
* Updates X rotation.
*/
void us3DSceneSlicing::updateRotX(int x)
{
  double normal[3];
  plane1->GetNormal(normal);
  normal[0] = x / 100.0 - 0.5;
  plane1->SetNormal(normal);
  view->update();
}

/**
* Updates Y rotation.
*/
void us3DSceneSlicing::updateRotY(int y)
{
  double normal[3];
  plane1->GetNormal(normal);
  normal[1] = y / 100.0 - 0.50;
  plane1->SetNormal(normal);
  view->update();
}

/**
* Updates Y rotation.
*/
void us3DSceneSlicing::updateRotZ(int z)
{
  double normal[3];
  plane1->GetNormal(normal);
  normal[2] = z / 100.0 - 0.50;
  plane1->SetNormal(normal);
  double origin[3];
  plane1->GetOrigin(origin);
  view->update();
}

#endif
