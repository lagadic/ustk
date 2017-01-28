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

#include <visp3/ustk_gui/usResliceImageViewer.h>

#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkImageActor.h>
#include <vtkResliceCursorWidget.h>
#include <vtkResliceCursorLineRepresentation.h>
#include <vtkResliceCursorThickLineRepresentation.h>
#include <vtkResliceCursorActor.h>
#include <vtkResliceCursorPolyDataAlgorithm.h>
#include <vtkPlane.h>
#include <vtkResliceCursor.h>
#include <vtkImageData.h>
#include <vtkImageMapToWindowLevelColors.h>
#include <vtkInteractorStyleImage.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkImageReslice.h>
#include <vtkScalarsToColors.h>
#include <vtkBoundedPlanePointPlacer.h>
#include <vtkPlane.h>
#include <vtkMath.h>
#include <vtkResliceImageViewerMeasurements.h>

vtkStandardNewMacro(usResliceImageViewer);

//----------------------------------------------------------------------------
// This class is used to scroll slices with the scroll bar. In the case of MPR
// view, it moves one "normalized spacing" in the direction of the normal to
// the resliced plane, provided the new center will continue to lie within the
// volume.
/*class vtkResliceImageViewerScrollCallback : public vtkCommand
{
public:
  static vtkResliceImageViewerScrollCallback *New()
    { return new vtkResliceImageViewerScrollCallback; }

  void Execute(vtkObject *, unsigned long ev, void*) VTK_OVERRIDE
  {
    if (!this->Viewer->GetSliceScrollOnMouseWheel())
    {
      return;
    }

    // Do not process if any modifiers are ON
    if (this->Viewer->GetInteractor()->GetShiftKey() ||
        this->Viewer->GetInteractor()->GetControlKey() ||
        this->Viewer->GetInteractor()->GetAltKey())
    {
      return;
    }

    // forwards or backwards
    int sign = (ev == vtkCommand::MouseWheelForwardEvent) ? 1 : -1;
    this->Viewer->IncrementSlice(sign);

    // Abort further event processing for the scroll.
    this->SetAbortFlag(1);
  }

  vtkResliceImageViewerScrollCallback():Viewer(0) {}
  vtkResliceImageViewer *Viewer;
};*/

//----------------------------------------------------------------------------
usResliceImageViewer::usResliceImageViewer()
{
  // Default is to not use the reslice cursor widget, ie use fast
  // 3D texture mapping to display slices.
  this->ResliceMode = usResliceImageViewer::RESLICE_AXIS_ALIGNED;

  // Set up the reslice cursor widget, should it be used.

  this->ResliceCursorWidget = vtkResliceCursorWidget::New();

  vtkSmartPointer< vtkResliceCursor > resliceCursor =
    vtkSmartPointer< vtkResliceCursor >::New();
  resliceCursor->SetThickMode(0);
  resliceCursor->SetThickness(10, 10, 10);

  vtkSmartPointer< vtkResliceCursorLineRepresentation >
    resliceCursorRep = vtkSmartPointer<
      vtkResliceCursorLineRepresentation >::New();
  resliceCursorRep->GetResliceCursorActor()->
      GetCursorAlgorithm()->SetResliceCursor(resliceCursor);
  resliceCursorRep->GetResliceCursorActor()->
      GetCursorAlgorithm()->SetReslicePlaneNormal(this->SliceOrientation);
  this->ResliceCursorWidget->SetRepresentation(resliceCursorRep);

  this->PointPlacer = vtkBoundedPlanePointPlacer::New();

  /*this->Measurements = vtkResliceImageViewerMeasurements::New();
  this->Measurements->SetResliceImageViewer(this);*/

  //this->ScrollCallback = vtkResliceImageViewerScrollCallback::New();
  //this->ScrollCallback->Viewer = this;
  this->SliceScrollOnMouseWheel = 0;

  this->InstallPipeline();
}

//----------------------------------------------------------------------------
usResliceImageViewer::~usResliceImageViewer()
{
  this->Measurements->Delete();

  if (this->ResliceCursorWidget)
  {
    this->ResliceCursorWidget->Delete();
    this->ResliceCursorWidget = NULL;
  }

  this->PointPlacer->Delete();
  //this->ScrollCallback->Delete();
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetThickMode( int t )
{
  vtkSmartPointer< vtkResliceCursor > rc = this->GetResliceCursor();

  if (t == this->GetThickMode())
  {
    return;
  }

  vtkSmartPointer< vtkResliceCursorLineRepresentation >
    resliceCursorRepOld = vtkResliceCursorLineRepresentation::SafeDownCast(
                          this->ResliceCursorWidget->GetRepresentation());
  vtkSmartPointer< vtkResliceCursorLineRepresentation > resliceCursorRepNew;

  this->GetResliceCursor()->SetThickMode(t);

  if (t)
  {
    resliceCursorRepNew = vtkSmartPointer<
        vtkResliceCursorThickLineRepresentation >::New();
  }
  else
  {
    resliceCursorRepNew = vtkSmartPointer<
        vtkResliceCursorLineRepresentation >::New();
  }

  int e = this->ResliceCursorWidget->GetEnabled();
  this->ResliceCursorWidget->SetEnabled(0);

  resliceCursorRepNew->GetResliceCursorActor()->
      GetCursorAlgorithm()->SetResliceCursor(rc);
  resliceCursorRepNew->GetResliceCursorActor()->
      GetCursorAlgorithm()->SetReslicePlaneNormal(this->SliceOrientation);
  this->ResliceCursorWidget->SetRepresentation(resliceCursorRepNew);
  resliceCursorRepNew->SetLookupTable(resliceCursorRepOld->GetLookupTable());

  resliceCursorRepNew->SetWindowLevel(
      resliceCursorRepOld->GetWindow(),
      resliceCursorRepOld->GetLevel(), 1);

  this->ResliceCursorWidget->SetEnabled(e);
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetResliceCursor( vtkResliceCursor * rc )
{
  vtkResliceCursorRepresentation *rep =
    vtkResliceCursorRepresentation::SafeDownCast(
          this->GetResliceCursorWidget()->GetRepresentation());
  rep->GetCursorAlgorithm()->SetResliceCursor(rc);

  // Rehook the observer to this reslice cursor.
  //this->Measurements->SetResliceImageViewer(this);
}

//----------------------------------------------------------------------------
int usResliceImageViewer::GetThickMode()
{
  return (vtkResliceCursorThickLineRepresentation::
    SafeDownCast(this->ResliceCursorWidget->GetRepresentation())) ? 1 : 0;
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetLookupTable( vtkScalarsToColors * l )
{
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    rep->SetLookupTable(l);
  }

  if (this->WindowLevel)
  {
    this->WindowLevel->SetLookupTable(l);
    this->WindowLevel->SetOutputFormatToRGBA();
    this->WindowLevel->PassAlphaToOutputOn();
  }
}

//----------------------------------------------------------------------------
vtkScalarsToColors * usResliceImageViewer::GetLookupTable()
{
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    return rep->GetLookupTable();
  }

  return NULL;
}

//----------------------------------------------------------------------------
void usResliceImageViewer::UpdateOrientation()
{
    // Set the camera position

    vtkCamera *cam = this->Renderer ? this->Renderer->GetActiveCamera() : NULL;
    if (cam)
    {
      switch (this->SliceOrientation)
      {
        case usImageViewer::SLICE_ORIENTATION_XY:
          cam->SetFocalPoint(0,0,0);
          cam->SetPosition(0,0,1); // -1 if medical ?
          cam->SetViewUp(0,0,1);
          break;

        case usImageViewer::SLICE_ORIENTATION_XZ:
          cam->SetFocalPoint(0,0,0);
          cam->SetPosition(0,1,0); // 1 if medical ?
          cam->SetViewUp(0,1,0);
          break;

        case usImageViewer::SLICE_ORIENTATION_YZ:
          cam->SetFocalPoint(0,0,0);
          cam->SetPosition(1,0,0); // -1 if medical ?
          cam->SetViewUp(1,0,0);
          break;
      }
    }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::UpdateDisplayExtent()
{
  // Only update the display extent in axis aligned mode

  if (this->ResliceMode == RESLICE_AXIS_ALIGNED)
  {
    this->Superclass::UpdateDisplayExtent();
    switch (this->SliceOrientation)
    {
    case usImageViewer::SLICE_ORIENTATION_XY:
      this->ImageActor->SetDisplayExtent(
        1000,1000,1000,1000, this->Slice, this->Slice);
      break;

    case usImageViewer::SLICE_ORIENTATION_XZ:
      this->ImageActor->SetDisplayExtent(
        1000,1000,this->Slice, this->Slice, 1000,1000);
      break;

    case usImageViewer::SLICE_ORIENTATION_YZ:
      this->ImageActor->SetDisplayExtent(
        this->Slice, this->Slice, 1000,1000,1000,1000);
      break;
    }

    /*vtkCamera *cam = this->Renderer->GetActiveCamera();
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
          range - avg_spacing * 3.0, range + avg_spacing * 3.0);*/

  }
  else {
  std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<< std::endl;
    switch (this->SliceOrientation)
    {
    case usImageViewer::SLICE_ORIENTATION_XY:
      this->ImageActor->SetDisplayExtent(
        1000,1000,1000,1000, this->Slice, this->Slice);
      break;

    case usImageViewer::SLICE_ORIENTATION_XZ:
      this->ImageActor->SetDisplayExtent(
        1000,1000,this->Slice, this->Slice, 1000,1000);
      break;

    case usImageViewer::SLICE_ORIENTATION_YZ:
      this->ImageActor->SetDisplayExtent(
        this->Slice, this->Slice, 1000,1000,1000,1000);
      break;
    }
  }

}

//----------------------------------------------------------------------------
void usResliceImageViewer::InstallPipeline()
{
  this->Superclass::InstallPipeline();

  if (this->Interactor)
  {
    this->ResliceCursorWidget->SetInteractor(this->Interactor);

    // Observe the scroll for slice manipulation at a higher priority
    // than the interactor style.
    //this->Interactor->RemoveObserver(this->ScrollCallback);
    //this->Interactor->AddObserver(vtkCommand::MouseWheelForwardEvent,
        //this->ScrollCallback, 0.55 );
    //this->Interactor->AddObserver(vtkCommand::MouseWheelBackwardEvent,
        //this->ScrollCallback, 0.55 );
  }

  if (this->Renderer)
  {
    this->ResliceCursorWidget->SetDefaultRenderer(this->Renderer);
    vtkCamera *cam = this->Renderer->GetActiveCamera();
    cam->ParallelProjectionOn();
  }

  if (this->ResliceMode == RESLICE_OBLIQUE)
  {
    this->ResliceCursorWidget->SetEnabled(1);
    this->ImageActor->SetVisibility(0);
    this->UpdateOrientation();

    double bounds[6] = {0, 1, 0, 1, 0, 1};

    vtkCamera *cam = this->Renderer->GetActiveCamera();
    double onespacing[3] = {1, 1, 1};
    double *spacing = onespacing;
    if (this->GetResliceCursor()->GetImage())
    {
      this->GetResliceCursor()->GetImage()->GetBounds(bounds);
      spacing = this->GetResliceCursor()->GetImage()->GetSpacing();
    }
    double avg_spacing =
      (spacing[0] + spacing[1] + spacing[2]) / 3.0;
    cam->SetClippingRange(
      bounds[this->SliceOrientation * 2] - 100 * avg_spacing,
      bounds[this->SliceOrientation * 2 + 1] + 100 * avg_spacing);
  }
  else
  {
    this->ResliceCursorWidget->SetEnabled(0);
    this->ImageActor->SetVisibility(1);
    this->UpdateOrientation();
  }

  if (this->WindowLevel)
  {
    this->WindowLevel->SetLookupTable(this->GetLookupTable());
  }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::UnInstallPipeline()
{
  this->ResliceCursorWidget->SetEnabled(0);

  if (this->Interactor)
  {
    //this->Interactor->RemoveObserver(this->ScrollCallback);
  }

  this->Superclass::UnInstallPipeline();
}

//----------------------------------------------------------------------------
void usResliceImageViewer::UpdatePointPlacer()
{
  std::cout << "usResliceImageViewer::UpdatePointPlacer()" << std::endl;
  if (this->ResliceMode == RESLICE_OBLIQUE)
  {
    this->PointPlacer->SetProjectionNormalToOblique();
    if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
    {
      const int planeOrientation =
        rep->GetCursorAlgorithm()->GetReslicePlaneNormal();
      vtkPlane *plane = this->GetResliceCursor()->GetPlane(planeOrientation);
      //plane->Print(std::cout);
      this->PointPlacer->SetObliquePlane(plane);
    }
  }
  else
  {

    if (!this->WindowLevel->GetInput())
    {
      return;
    }

    vtkImageData *input = this->ImageActor->GetInput();
    if ( !input )
    {
      return;
    }

    double spacing[3];
    input->GetSpacing(spacing);

    double origin[3];
    input->GetOrigin(origin);

    double bounds[6];
    this->ImageActor->GetBounds(bounds);

    int displayExtent[6];
    this->ImageActor->GetDisplayExtent(displayExtent);

    int axis = vtkBoundedPlanePointPlacer::XAxis;
    double position = 0.0;
    if ( displayExtent[0] == displayExtent[1] )
    {
      axis = vtkBoundedPlanePointPlacer::XAxis;
      position = origin[0] + displayExtent[0]*spacing[0];
    }
    else if ( displayExtent[2] == displayExtent[3] )
    {
      axis = vtkBoundedPlanePointPlacer::YAxis;
      position = origin[1] + displayExtent[2]*spacing[1];
    }
    else if ( displayExtent[4] == displayExtent[5] )
    {
      axis = vtkBoundedPlanePointPlacer::ZAxis;
      position = origin[2] + displayExtent[4]*spacing[2];
    }

    this->PointPlacer->SetProjectionNormal(axis);
    this->PointPlacer->SetProjectionPosition(position);
  }

}

//----------------------------------------------------------------------------
void usResliceImageViewer::Render()
{
  std::cout << "usResliceImageViewer::Render()" << std::endl;
  if (!this->WindowLevel->GetInput())
  {
    return;
  }

  this->UpdatePointPlacer();

  this->Superclass::Render();
}

//----------------------------------------------------------------------------
vtkResliceCursor * usResliceImageViewer::GetResliceCursor()
{
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    return rep->GetResliceCursor();
  }

  return NULL;
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetInputData(vtkImageData *in)
{
  if(!in)
  {
    return;
  }

  this->WindowLevel->SetInputData(in);
  this->GetResliceCursor()->SetImage(in);
  this->GetResliceCursor()->SetCenter(in->GetCenter());
  this->UpdateDisplayExtent();

  double range[2];
  in->GetScalarRange(range);
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    if (vtkImageReslice *reslice =
        vtkImageReslice::SafeDownCast(rep->GetReslice()))
    {
      // default background color is the min value of the image scalar range
      reslice->SetBackgroundColor(range[0],range[0],range[0],range[0]);
      this->SetColorWindow(range[1]-range[0]);
      this->SetColorLevel((range[0]+range[1])/2.0);
    }
  }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetInputConnection(vtkAlgorithmOutput* input)
{
  vtkErrorMacro( << "Use SetInputData instead. " );
  this->WindowLevel->SetInputConnection(input);
  this->UpdateDisplayExtent();
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetResliceMode( int r )
{
  if (r == this->ResliceMode)
  {
    return;
  }

  this->ResliceMode = r;
  this->Modified();

  this->InstallPipeline();
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetColorWindow( double w )
{
  double rmin = this->GetColorLevel() - 0.5*fabs( w );
  double rmax = rmin + fabs( w );
  this->GetLookupTable()->SetRange( rmin, rmax );

  this->WindowLevel->SetWindow(w);
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    rep->SetWindowLevel(w, rep->GetLevel(), 1);
  }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::SetColorLevel( double w )
{
  double rmin = w - 0.5*fabs( this->GetColorWindow() );
  double rmax = rmin + fabs( this->GetColorWindow() );
  this->GetLookupTable()->SetRange( rmin, rmax );

  this->WindowLevel->SetLevel(w);
  if (vtkResliceCursorRepresentation *rep =
        vtkResliceCursorRepresentation::SafeDownCast(
          this->ResliceCursorWidget->GetRepresentation()))
  {
    rep->SetWindowLevel(rep->GetWindow(), w, 1);
  }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::Reset()
{
  this->ResliceCursorWidget->ResetResliceCursor();
}

//----------------------------------------------------------------------------
vtkPlane * usResliceImageViewer::GetReslicePlane()
{
  // Get the reslice plane
  if (vtkResliceCursorRepresentation *rep =
      vtkResliceCursorRepresentation::SafeDownCast(
        this->ResliceCursorWidget->GetRepresentation()))
  {
    const int planeOrientation =
      rep->GetCursorAlgorithm()->GetReslicePlaneNormal();
    vtkPlane *plane = this->GetResliceCursor()->GetPlane(planeOrientation);
    return plane;
  }

  return NULL;
}

//----------------------------------------------------------------------------
double usResliceImageViewer::GetInterSliceSpacingInResliceMode()
{
  double n[3], imageSpacing[3], resliceSpacing = 0;

  if (vtkPlane *plane = this->GetReslicePlane())
  {
    plane->GetNormal(n);
    this->GetResliceCursor()->GetImage()->GetSpacing(imageSpacing);
    resliceSpacing = fabs(vtkMath::Dot(n, imageSpacing));
  }

  return resliceSpacing;
}

//----------------------------------------------------------------------------
void usResliceImageViewer::IncrementSlice( int inc )
{
  if (this->GetResliceMode() ==
      usResliceImageViewer::RESLICE_AXIS_ALIGNED)
  {
    int oldSlice = this->GetSlice();
    this->SetSlice(this->GetSlice() + inc);
    if (this->GetSlice() != oldSlice)
    {
      this->InvokeEvent( usResliceImageViewer::SliceChangedEvent, NULL );
      this->InvokeEvent( vtkCommand::InteractionEvent, NULL );
    }
  }
  else
  {
    if (vtkPlane *p = this->GetReslicePlane())
    {
      double n[3], c[3], bounds[6];
      p->GetNormal(n);
      const double spacing =
          this->GetInterSliceSpacingInResliceMode() * inc;
      this->GetResliceCursor()->GetCenter(c);
      vtkMath::MultiplyScalar(n, spacing);
      c[0] += n[0];
      c[1] += n[1];
      c[2] += n[2];

      // If the new center is inside, put it there...
      if (vtkImageData *image = this->GetResliceCursor()->GetImage())
      {
        image->GetBounds(bounds);
        if (c[0] >= bounds[0] && c[0] <= bounds[1] &&
            c[1] >= bounds[2] && c[1] <= bounds[3] &&
            c[2] >= bounds[4] && c[2] <= bounds[5])
        {
          this->GetResliceCursor()->SetCenter(c);

          this->InvokeEvent( usResliceImageViewer::SliceChangedEvent, NULL );
          this->InvokeEvent( vtkCommand::InteractionEvent, NULL );
        }
      }
    }
  }
}

//----------------------------------------------------------------------------
void usResliceImageViewer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "ResliceCursorWidget:\n";
  this->ResliceCursorWidget->PrintSelf(os,indent.GetNextIndent());
  os << indent << "ResliceMode: " << this->ResliceMode << endl;
  os << indent << "SliceScrollOnMouseWheel: " << this->SliceScrollOnMouseWheel << endl;
  os << indent << "Point Placer: ";
  this->PointPlacer->PrintSelf(os,indent.GetNextIndent());
  os << indent << "Measurements: ";
  this->Measurements->PrintSelf(os,indent.GetNextIndent());
  os << indent << "Interactor: " << this->Interactor << "\n";
  if (this->Interactor)
  {
    this->Interactor->PrintSelf(os,indent.GetNextIndent());
  }
}
#endif
