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
 * @file us3DSceneWidget.h
 * @brief Class used to render a 3D vtk scene containing a vtkImageData in a QWidget (based on QVTKWidget)
 */


#ifndef __us3DSceneWidget_h_
#define __us3DSceneWidget_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usViewerWidget.h>

//VTK includes
#include <vtkPlane.h>
#include <vtkImageSlice.h>
#include <vtkImageResliceMapper.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

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
 * @class us3DSceneWidget
 * @brief Class used to render a 3D vtk scene containing a vtkImageData in a QWidget (based on QVTKWidget).
 * @ingroup module_ustk_gui
 *
 * Usage :
 *    \code
#include <visp3/ustk_gui/us3DSceneWidget.h>

int main()
{
  usImagePostScan3D<unsigned char> postScan3D;
  usImageIo::read(postScan3D, mhd_filename);
  //conversion to vtk format
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
  usVTKConverter::convert(postScan3D,vtkImage);

  //setup view widget
  us3DSceneWidget scene;
  scene.setImageData(vtkImage);

  vtkSmartPointer<vtkPlane> planeX = vtkSmartPointer<vtkPlane>::New();
  planeX->SetNormal(1,0,0);
  planeX->SetOrigin(postScan3D.getDimX()/2,0,0);
  vtkSmartPointer<vtkPlane> planeY = vtkSmartPointer<vtkPlane>::New();
  planeY->SetNormal(0,1,0);
  planeY->SetOrigin(0,postScan3D.getDimY()/2,0);
  vtkSmartPointer<vtkPlane> planeZ = vtkSmartPointer<vtkPlane>::New();
  planeZ->SetNormal(0,0,1);
  planeZ->SetOrigin(0,0,postScan3D.getDimZ()/2);
  scene.setPlanes(planeX,planeY,planeZ);

  scene.init();
  scene.show();
}
  \endcode
 */

class VISP_EXPORT us3DSceneWidget : public usViewerWidget
{
  Q_OBJECT
public:

  // Constructor/Destructor
  us3DSceneWidget(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  ~us3DSceneWidget() {}

  vtkImageData* getImageData();

  //Catch paint events, in case we want to display some informations (writing in this widget) over the vtk scene
  void paintEvent( QPaintEvent* event );

  void init();

  //Set image to display
  void setImageData(vtkImageData* imageData);

  //Set planes
  void setPlane1(vtkPlane* plane);
  void setPlane2(vtkPlane* plane);
  void setPlane3(vtkPlane* plane);
  void setPlanes(vtkPlane* plane1,vtkPlane* plane2,vtkPlane* plane3);

public slots:
  //
  void updateImageData(vtkImageData* imageData);

private:
  //image
  vtkImageData* imageData;

  //planes (containing geometrical informations)
  vtkPlane* plane1;
  vtkPlane* plane2;
  vtkPlane* plane3;

  //vtk mappers
  vtkImageResliceMapper* imageResliceMapper1;
  vtkImageResliceMapper* imageResliceMapper2;
  vtkImageResliceMapper* imageResliceMapper3;

  //vtk actors
  vtkImageSlice* imageSlice1;
  vtkImageSlice* imageSlice2;
  vtkImageSlice* imageSlice3;

  //vtk renderer
  vtkRenderer* renderer;

};
#endif
#endif // __us3DSceneWidget_h_
