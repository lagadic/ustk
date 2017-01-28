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


#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usImageViewer.h>

#include "vtkCamera.h"
#include "vtkCommand.h"
#include "vtkImageActor.h"
#include "vtkImageData.h"
#include "vtkImageMapper3D.h"
#include "vtkImageMapToWindowLevelColors.h"
#include "vtkInformation.h"
#include "vtkInteractorStyleImage.h"
#include "vtkObjectFactory.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStreamingDemandDrivenPipeline.h"

vtkStandardNewMacro(usImageViewer);

//----------------------------------------------------------------------------
usImageViewer::usImageViewer()
{
  this->RenderWindow    = NULL;
  this->Renderer        = NULL;
  this->ImageActor      = vtkImageActor::New();
  this->WindowLevel     = vtkImageMapToWindowLevelColors::New();
  this->Interactor      = NULL;
  this->InteractorStyle = NULL;

  this->Slice = 0;
  this->FirstRender = 1;
  this->SliceOrientation = usImageViewer::SLICE_ORIENTATION_XY;

  // Setup the pipeline

  vtkRenderWindow *renwin = vtkRenderWindow::New();
  this->SetRenderWindow(renwin);
  renwin->Delete();

  vtkRenderer *ren = vtkRenderer::New();
  this->SetRenderer(ren);
  ren->Delete();

  this->InstallPipeline();
}

//----------------------------------------------------------------------------
usImageViewer::~usImageViewer()
{
  if (this->WindowLevel)
  {
    this->WindowLevel->Delete();
    this->WindowLevel = NULL;
  }

  if (this->ImageActor)
  {
    this->ImageActor->Delete();
    this->ImageActor = NULL;
  }

  if (this->Renderer)
  {
    this->Renderer->Delete();
    this->Renderer = NULL;
  }

  if (this->RenderWindow)
  {
    this->RenderWindow->Delete();
    this->RenderWindow = NULL;
  }

  if (this->Interactor)
  {
    this->Interactor->Delete();
    this->Interactor = NULL;
  }

  if (this->InteractorStyle)
  {
    this->InteractorStyle->Delete();
    this->InteractorStyle = NULL;
  }
}

//----------------------------------------------------------------------------
void usImageViewer::SetupInteractor(vtkRenderWindowInteractor *arg)
{
  if (this->Interactor == arg)
  {
    return;
  }

  this->UnInstallPipeline();

  if (this->Interactor)
  {
    this->Interactor->UnRegister(this);
  }

  this->Interactor = arg;

  if (this->Interactor)
  {
    this->Interactor->Register(this);
  }

  this->InstallPipeline();

  if (this->Renderer)
  {
    this->Renderer->GetActiveCamera()->ParallelProjectionOn();
  }
}

//----------------------------------------------------------------------------
void usImageViewer::SetRenderWindow(vtkRenderWindow *arg)
{
  if (this->RenderWindow == arg)
  {
    return;
  }

  this->UnInstallPipeline();

  if (this->RenderWindow)
  {
    this->RenderWindow->UnRegister(this);
  }

  this->RenderWindow = arg;

  if (this->RenderWindow)
  {
    this->RenderWindow->Register(this);
  }

  this->InstallPipeline();
}

//----------------------------------------------------------------------------
void usImageViewer::SetRenderer(vtkRenderer *arg)
{
  if (this->Renderer == arg)
  {
    return;
  }

  this->UnInstallPipeline();

  if (this->Renderer)
  {
    this->Renderer->UnRegister(this);
  }

  this->Renderer = arg;

  if (this->Renderer)
  {
    this->Renderer->Register(this);
  }

  this->InstallPipeline();
  this->UpdateOrientation();
}

//----------------------------------------------------------------------------
void usImageViewer::SetSize(int a,int b)
{
  this->RenderWindow->SetSize(a, b);
}

//----------------------------------------------------------------------------
int* usImageViewer::GetSize()
{
  return this->RenderWindow->GetSize();
}

//----------------------------------------------------------------------------
void usImageViewer::GetSliceRange(int &min, int &max)
{
  vtkAlgorithm *input = this->GetInputAlgorithm();
  if (input)
  {
    input->UpdateInformation();
    int *w_ext = input->GetOutputInformation(0)->Get(
      vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT());
    min = w_ext[this->SliceOrientation * 2];
    max = w_ext[this->SliceOrientation * 2 + 1];
  }
}

//----------------------------------------------------------------------------
int* usImageViewer::GetSliceRange()
{
  vtkAlgorithm *input = this->GetInputAlgorithm();
  if (input)
  {
    input->UpdateInformation();
    return input->GetOutputInformation(0)->Get(
      vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT()) +
      this->SliceOrientation * 2;
  }
  return NULL;
}

//----------------------------------------------------------------------------
int usImageViewer::GetSliceMin()
{
  int *range = this->GetSliceRange();
  if (range)
  {
    return range[0];
  }
  return 0;
}

//----------------------------------------------------------------------------
int usImageViewer::GetSliceMax()
{
  int *range = this->GetSliceRange();
  if (range)
  {
    return range[1];
  }
  return 0;
}

//----------------------------------------------------------------------------
void usImageViewer::SetSlice(int slice)
{
  int *range = this->GetSliceRange();
  if (range)
  {
    if (slice < range[0])
    {
      slice = range[0];
    }
    else if (slice > range[1])
    {
      slice = range[1];
    }
  }

  if (this->Slice == slice)
  {
    return;
  }

  this->Slice = slice;
  this->Modified();

  this->UpdateDisplayExtent();
  this->Render();
}

//----------------------------------------------------------------------------
void usImageViewer::SetSliceOrientation(int orientation)
{
  if (orientation < usImageViewer::SLICE_ORIENTATION_YZ ||
      orientation > usImageViewer::SLICE_ORIENTATION_XY)
  {
    vtkErrorMacro("Error - invalid slice orientation " << orientation);
    return;
  }

  if (this->SliceOrientation == orientation)
  {
    return;
  }

  this->SliceOrientation = orientation;

  // Update the viewer

  int *range = this->GetSliceRange();
  if (range)
  {
    this->Slice = static_cast<int>((range[0] + range[1]) * 0.5);
  }

  this->UpdateOrientation();
  this->UpdateDisplayExtent();

  if (this->Renderer && this->GetInput())
  {
    double scale = this->Renderer->GetActiveCamera()->GetParallelScale();
    this->Renderer->ResetCamera();
    this->Renderer->GetActiveCamera()->SetParallelScale(scale);
  }

  this->Render();
}

//----------------------------------------------------------------------------
void usImageViewer::UpdateOrientation()
{
  // Set the camera position
  std::cout << "usImageViewer::UpdateOrientation()" << std::endl;

  vtkCamera *cam = this->Renderer ? this->Renderer->GetActiveCamera() : NULL;
  if (cam)
  {
    switch (this->SliceOrientation)
    {
      case usImageViewer::SLICE_ORIENTATION_XY:
        cam->SetFocalPoint(0,0,0);
        cam->SetPosition(0,0,1); // -1 if medical ?
        cam->SetViewUp(0,1,0);
        break;

      case usImageViewer::SLICE_ORIENTATION_XZ:
        cam->SetFocalPoint(0,0,0);
        cam->SetPosition(0,1,0); // 1 if medical ?
        cam->SetViewUp(0,0,1);
        break;

      case usImageViewer::SLICE_ORIENTATION_YZ:
        cam->SetFocalPoint(0,0,0);
        cam->SetPosition(1,0,0); // -1 if medical ?
        cam->SetViewUp(0,0,1);
        break;
    }
  }
}

//----------------------------------------------------------------------------
void usImageViewer::UpdateDisplayExtent()
{
  std::cout << "usImageViewer::UpdateDisplayExtent()" << std::endl;
  vtkAlgorithm *input = this->GetInputAlgorithm();
  if (!input || !this->ImageActor)
  {
    return;
  }

  input->UpdateInformation();
  vtkInformation* outInfo = input->GetOutputInformation(0);
  int *w_ext = outInfo->Get(
    vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT());

  //std::cout << "extend : " <<   w_ext[0] << "," << w_ext[1] << "," << w_ext[2] << "," << w_ext[3] << "," << w_ext[4] << "," << w_ext[5] << std::endl;

  // Is the slice in range ? If not, fix it

  int slice_min = w_ext[this->SliceOrientation * 2];
  int slice_max = w_ext[this->SliceOrientation * 2 + 1];
  if (this->Slice < slice_min || this->Slice > slice_max)
  {
    this->Slice = static_cast<int>((slice_min + slice_max) * 0.5);
  }

  // Set the image actor

  switch (this->SliceOrientation)
  {
    case usImageViewer::SLICE_ORIENTATION_XY:
      this->ImageActor->SetDisplayExtent(
        w_ext[0]*2, w_ext[1]*2, w_ext[2]*2, w_ext[3]*2, this->Slice, this->Slice);
      break;

    case usImageViewer::SLICE_ORIENTATION_XZ:
      this->ImageActor->SetDisplayExtent(
        w_ext[0]*2, w_ext[1]*2, this->Slice, this->Slice, w_ext[4]*2, w_ext[5]*2);
      break;

    case usImageViewer::SLICE_ORIENTATION_YZ:
      this->ImageActor->SetDisplayExtent(
        this->Slice, this->Slice, w_ext[2]*2, w_ext[3]*2, w_ext[4]*2, w_ext[5]*2);
      break;
  }

  // Figure out the correct clipping range

  if (this->Renderer)
  {
    if (this->InteractorStyle &&
        this->InteractorStyle->GetAutoAdjustCameraClippingRange())
    {
      this->Renderer->ResetCameraClippingRange();
    }
    else
    {
      vtkCamera *cam = this->Renderer->GetActiveCamera();
      if (cam)
      {
        double bounds[6];
        this->ImageActor->GetBounds(bounds);
        double spos = bounds[this->SliceOrientation * 2];
        double cpos = cam->GetPosition()[this->SliceOrientation];
        double range = fabs(spos - cpos);
        double *spacing = outInfo->Get(vtkDataObject::SPACING());
        double avg_spacing =
          (spacing[0] + spacing[1] + spacing[2]) / 3.0;
        cam->SetClippingRange(
          (range - avg_spacing * 3.0)*2, (range + avg_spacing * 3.0)*2);
      }
    }
  }
}

//----------------------------------------------------------------------------
void usImageViewer::SetPosition(int a,int b)
{
  this->RenderWindow->SetPosition(a, b);
}

//----------------------------------------------------------------------------
int* usImageViewer::GetPosition()
{
  return this->RenderWindow->GetPosition();
}

//----------------------------------------------------------------------------
void usImageViewer::SetDisplayId(void *a)
{
  this->RenderWindow->SetDisplayId(a);
}

//----------------------------------------------------------------------------
void usImageViewer::SetWindowId(void *a)
{
  this->RenderWindow->SetWindowId(a);
}

//----------------------------------------------------------------------------
void usImageViewer::SetParentId(void *a)
{
  this->RenderWindow->SetParentId(a);
}

//----------------------------------------------------------------------------
double usImageViewer::GetColorWindow()
{
  return this->WindowLevel->GetWindow();
}

//----------------------------------------------------------------------------
double usImageViewer::GetColorLevel()
{
  return this->WindowLevel->GetLevel();
}

//----------------------------------------------------------------------------
void usImageViewer::SetColorWindow(double s)
{
  this->WindowLevel->SetWindow(s);
}

//----------------------------------------------------------------------------
void usImageViewer::SetColorLevel(double s)
{
  this->WindowLevel->SetLevel(s);
}

//----------------------------------------------------------------------------
class usImageViewerCallback : public vtkCommand
{
public:
  static usImageViewerCallback *New() { return new usImageViewerCallback; }

  void Execute(vtkObject *caller,
               unsigned long event,
               void *vtkNotUsed(callData))
  {
      if (this->IV->GetInput() == NULL)
      {
        return;
      }

      // Reset

      if (event == vtkCommand::ResetWindowLevelEvent)
      {
        this->IV->GetInputAlgorithm()->UpdateWholeExtent();
        double *range = this->IV->GetInput()->GetScalarRange();
        this->IV->SetColorWindow(range[1] - range[0]);
        this->IV->SetColorLevel(0.5 * (range[1] + range[0]));
        this->IV->Render();
        return;
      }

      // Start

      if (event == vtkCommand::StartWindowLevelEvent)
      {
        this->InitialWindow = this->IV->GetColorWindow();
        this->InitialLevel = this->IV->GetColorLevel();
        return;
      }

      // Adjust the window level here

      vtkInteractorStyleImage *isi =
        static_cast<vtkInteractorStyleImage *>(caller);

      int *size = this->IV->GetRenderWindow()->GetSize();
      double window = this->InitialWindow;
      double level = this->InitialLevel;

      // Compute normalized delta

      double dx = 4.0 *
        (isi->GetWindowLevelCurrentPosition()[0] -
         isi->GetWindowLevelStartPosition()[0]) / size[0];
      double dy = 4.0 *
        (isi->GetWindowLevelStartPosition()[1] -
         isi->GetWindowLevelCurrentPosition()[1]) / size[1];

      // Scale by current values

      if (fabs(window) > 0.01)
      {
        dx = dx * window;
      }
      else
      {
        dx = dx * (window < 0 ? -0.01 : 0.01);
      }
      if (fabs(level) > 0.01)
      {
        dy = dy * level;
      }
      else
      {
        dy = dy * (level < 0 ? -0.01 : 0.01);
      }

      // Abs so that direction does not flip

      if (window < 0.0)
      {
        dx = -1*dx;
      }
      if (level < 0.0)
      {
        dy = -1*dy;
      }

      // Compute new window level

      double newWindow = dx + window;
      double newLevel;
      newLevel = level - dy;

      // Stay away from zero and really

      if (fabs(newWindow) < 0.01)
      {
        newWindow = 0.01*(newWindow < 0 ? -1 : 1);
      }
      if (fabs(newLevel) < 0.01)
      {
        newLevel = 0.01*(newLevel < 0 ? -1 : 1);
      }

      this->IV->SetColorWindow(newWindow);
      this->IV->SetColorLevel(newLevel);
      this->IV->Render();
  }

  usImageViewer *IV;
  double InitialWindow;
  double InitialLevel;
};

//----------------------------------------------------------------------------
void usImageViewer::InstallPipeline()
{
  if (this->RenderWindow && this->Renderer)
  {
    this->RenderWindow->AddRenderer(this->Renderer);
  }

  if (this->Interactor)
  {
    if (!this->InteractorStyle)
    {
      this->InteractorStyle = vtkInteractorStyleImage::New();
      usImageViewerCallback *cbk = usImageViewerCallback::New();
      cbk->IV = this;
      this->InteractorStyle->AddObserver(
        vtkCommand::WindowLevelEvent, cbk);
      this->InteractorStyle->AddObserver(
        vtkCommand::StartWindowLevelEvent, cbk);
      this->InteractorStyle->AddObserver(
        vtkCommand::ResetWindowLevelEvent, cbk);
      cbk->Delete();
    }

    this->Interactor->SetInteractorStyle(this->InteractorStyle);
    this->Interactor->SetRenderWindow(this->RenderWindow);
  }

  if (this->Renderer && this->ImageActor)
  {
    this->Renderer->AddViewProp(this->ImageActor);
  }

  if (this->ImageActor && this->WindowLevel)
  {
    this->ImageActor->GetMapper()->SetInputConnection(
      this->WindowLevel->GetOutputPort());
  }
}

//----------------------------------------------------------------------------
void usImageViewer::UnInstallPipeline()
{
  if (this->ImageActor)
  {
    this->ImageActor->GetMapper()->SetInputConnection(NULL);
  }

  if (this->Renderer && this->ImageActor)
  {
    this->Renderer->RemoveViewProp(this->ImageActor);
  }

  if (this->RenderWindow && this->Renderer)
  {
    this->RenderWindow->RemoveRenderer(this->Renderer);
  }

  if (this->Interactor)
  {
    this->Interactor->SetInteractorStyle(NULL);
    this->Interactor->SetRenderWindow(NULL);
  }
}

//----------------------------------------------------------------------------
void usImageViewer::Render()
{
  if (this->FirstRender)
  {
    // Initialize the size if not set yet

    vtkAlgorithm *input = this->GetInputAlgorithm();
    if (input)
    {
      input->UpdateInformation();
      int *w_ext = this->GetInputInformation()->Get(
        vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT());
      int xs = 0, ys = 0;

      switch (this->SliceOrientation)
      {
        case usImageViewer::SLICE_ORIENTATION_XY:
        default:
          xs = w_ext[1] - w_ext[0] + 1;
          ys = w_ext[3] - w_ext[2] + 1;
          break;

        case usImageViewer::SLICE_ORIENTATION_XZ:
          xs = w_ext[1] - w_ext[0] + 1;
          ys = w_ext[5] - w_ext[4] + 1;
          break;

        case usImageViewer::SLICE_ORIENTATION_YZ:
          xs = w_ext[3] - w_ext[2] + 1;
          ys = w_ext[5] - w_ext[4] + 1;
          break;
      }

      // if it would be smaller than 150 by 100 then limit to 150 by 100
      if (this->RenderWindow->GetSize()[0] == 0)
      {
        this->RenderWindow->SetSize(
          xs < 150 ? 150 : xs, ys < 100 ? 100 : ys);
      }

      if (this->Renderer)
      {
        this->Renderer->ResetCamera();
        /*this->Renderer->GetActiveCamera()->SetParallelScale(
          xs < 150 ? 75 : (xs - 1 ) / 2.0);*/
      }
      this->FirstRender = 0;
    }
  }
  std::cout << "max x bound" << this->ImageActor->GetMaxXBound() << std::endl;
  std::cout << "min x bound" << this->ImageActor->GetMinXBound() << std::endl;
  std::cout << "max y bound" << this->ImageActor->GetMaxYBound() << std::endl;
  std::cout << "min y bound" << this->ImageActor->GetMinYBound() << std::endl;
  std::cout << "max z bound" << this->ImageActor->GetMaxZBound() << std::endl;
  std::cout << "min z bound" << this->ImageActor->GetMinZBound() << std::endl;


  int* ext = this->ImageActor->GetDisplayExtent();
  std::cout << "ext " << *ext << ext[1] << ext[2] << ext[3] << ext[4] << ext[5] << std::endl;
  if (this->GetInput())
  {
    this->RenderWindow->Render();
  }
}

//----------------------------------------------------------------------------
const char* usImageViewer::GetWindowName()
{
  return this->RenderWindow->GetWindowName();
}

//----------------------------------------------------------------------------
void usImageViewer::SetOffScreenRendering(int i)
{
  this->RenderWindow->SetOffScreenRendering(i);
}

//----------------------------------------------------------------------------
int usImageViewer::GetOffScreenRendering()
{
  return this->RenderWindow->GetOffScreenRendering();
}

//----------------------------------------------------------------------------
void usImageViewer::SetInputData(vtkImageData *in)
{
  this->WindowLevel->SetInputData(in);
  this->UpdateDisplayExtent();
}
//----------------------------------------------------------------------------
vtkImageData* usImageViewer::GetInput()
{
  return vtkImageData::SafeDownCast(this->WindowLevel->GetInput());
}
//----------------------------------------------------------------------------
vtkInformation* usImageViewer::GetInputInformation()
{
  return this->WindowLevel->GetInputInformation();
}
//----------------------------------------------------------------------------
vtkAlgorithm* usImageViewer::GetInputAlgorithm()
{
  return this->WindowLevel->GetInputAlgorithm();
}

//----------------------------------------------------------------------------
void usImageViewer::SetInputConnection(vtkAlgorithmOutput* input)
{
  this->WindowLevel->SetInputConnection(input);
  this->UpdateDisplayExtent();
}

//----------------------------------------------------------------------------
void usImageViewer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "RenderWindow:\n";
  this->RenderWindow->PrintSelf(os,indent.GetNextIndent());
  os << indent << "Renderer:\n";
  this->Renderer->PrintSelf(os,indent.GetNextIndent());
  os << indent << "ImageActor:\n";
  this->ImageActor->PrintSelf(os,indent.GetNextIndent());
  os << indent << "WindowLevel:\n" << endl;
  this->WindowLevel->PrintSelf(os,indent.GetNextIndent());
  os << indent << "Slice: " << this->Slice << endl;
  os << indent << "SliceOrientation: " << this->SliceOrientation << endl;
  os << indent << "InteractorStyle: " << endl;
  if (this->InteractorStyle)
  {
    os << "\n";
    this->InteractorStyle->PrintSelf(os,indent.GetNextIndent());
  }
  else
  {
    os << "None";
  }
}

#endif
