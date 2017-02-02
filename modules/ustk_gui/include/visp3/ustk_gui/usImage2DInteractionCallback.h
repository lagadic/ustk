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
 * @file usImage2DInteractionCallback.h
 * @brief Interaction callback for 2D slices
 */

#ifndef __usImage2DInteractionCallback_h_
#define __usImage2DInteractionCallback_h_

// VISP includes
#include <visp3/ustk_gui/usGuiConfig.h>
//#if 0
#ifdef USTK_HAVE_VTK_QT
#include <visp3/ustk_gui/us3DSceneWidget.h>

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageReslice.h>
#include <vtkCommand.h>

/**
 * @class usImage2DInteractionCallback
 * @brief Interaction callback for 2D slices
 * @ingroup module_ustk_gui
 */

class VISP_EXPORT usImage2DInteractionCallback : public vtkCommand
{
public:

  static usImage2DInteractionCallback *New() {
    return new usImage2DInteractionCallback; };

  usImage2DInteractionCallback() {
    this->ImageReslice = 0;
    this->Interactor = 0;
    this->plane = 0; }

  void SetPlane(vtkPlane *plane) {
    this->plane = plane; }

  void SetImageReslice(vtkImageReslice *reslice) {
    this->ImageReslice = reslice; }

  vtkImageReslice *GetImageReslice() {
    return this->ImageReslice; }

  void SetInteractor(vtkRenderWindowInteractor *interactor) {
    this->Interactor = interactor; }

  vtkRenderWindowInteractor *GetInteractor() {
    return this->Interactor; }

  void Execute(vtkObject *, unsigned long event, void *)
  {
    vtkRenderWindowInteractor *interactor = this->GetInteractor();

    if (event == vtkCommand::MouseWheelForwardEvent)
    {
        vtkImageReslice *reslice = this->ImageReslice;

        // Increment slice position by deltaY of mouse
        int deltaY = +1;

        reslice->Update();
        double sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
        vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
        // move the center point that we are slicing through
        double point[4];
        double center[4];
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = sliceSpacing * deltaY;
        point[3] = 1.0;
        matrix->MultiplyPoint(point, center);
        matrix->SetElement(0, 3, center[0]);
        matrix->SetElement(1, 3, center[1]);
        matrix->SetElement(2, 3, center[2]);
        //matrix->Print(std::cout);
        //widget3D->updatePlane1(center[0],center[1],center[2]);
        interactor->Render();
        std::cout << "callback updating plane origin : " << center[0] << "," << center[1] << "," << center[2] << std::endl;
        this->plane->SetOrigin(center[0],center[1],center[2]);
        widget3D->update();
        /*this->plane->Modified();
        this->plane->Print(std::cout);*/
      }
      else if (event == vtkCommand::MouseWheelBackwardEvent)
      {
       vtkImageReslice *reslice = this->ImageReslice;

        // Increment slice position by deltaY of mouse
        int deltaY = -1;

        reslice->Update();
        double sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
        vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
        // move the center point that we are slicing through
        double point[4];
        double center[4];
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = sliceSpacing * deltaY;
        point[3] = 1.0;
        matrix->MultiplyPoint(point, center);
        matrix->SetElement(0, 3, center[0]);
        matrix->SetElement(1, 3, center[1]);
        matrix->SetElement(2, 3, center[2]);
        //matrix->Print(std::cout);
        //widget3D->updatePlane1(center[0],center[1],center[2]);
        interactor->Render();
        std::cout << "callback updating plane origin : " << center[0] << "," << center[1] << "," << center[2] << std::endl;
        this->plane->SetOrigin(center[0],center[1],center[2]);
        widget3D->update();
        /*this->plane->Modified();
        this->plane->Print(std::cout);*/
      }
  }

  us3DSceneWidget* widget3D;
private:
  // Pointer to vtkImageReslice
  vtkImageReslice *ImageReslice;

  //Pointer to vtkPlane representing the view in the 3D scene
  vtkPlane* plane;

  // Pointer to the interactor
  vtkRenderWindowInteractor *Interactor;
};
#endif
//#endif
#endif // __usImage2DInteractionCallback_h_
