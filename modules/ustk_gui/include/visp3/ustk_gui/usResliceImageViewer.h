/**
 * @class   usResliceImageViewer
 * @brief   Display an image along with a reslice cursor
 *
 * Based on vtkResliceImageViewer
*/

#ifndef usResliceImageViewer_h
#define usResliceImageViewer_h

#include <visp3/ustk_gui/usGuiConfig.h>

//#include "vtkInteractionImageModule.h" // For export macro
#include "vtkImageViewer2.h"

class vtkResliceCursorWidget;
class vtkResliceCursor;
class vtkScalarsToColors;
class vtkBoundedPlanePointPlacer;
class vtkResliceImageViewerMeasurements;
class vtkResliceImageViewerScrollCallback;
class vtkPlane;

class VISP_EXPORT usResliceImageViewer : public vtkImageViewer2
{
public:

  //@{
  /**
   * Standard VTK methods.
   */
  static usResliceImageViewer *New();
  vtkTypeMacro(usResliceImageViewer,vtkImageViewer2);
  void PrintSelf(ostream& os, vtkIndent indent);
  //@}

  /**
   * Render the resulting image.
   */
  virtual void Render();

  //@{
  /**
   * Set/Get the input image to the viewer.
   */
  virtual void SetInputData(vtkImageData *in);
  virtual void SetInputConnection(vtkAlgorithmOutput* input);
  //@}

  //@{
  /**
   * Set window and level for mapping pixels to colors.
   */
  virtual void SetColorWindow(double s);
  virtual void SetColorLevel(double s);
  //@}

  //@{
  /**
   * Get the internal render window, renderer, image actor, and
   * image map instances.
   */
  vtkGetObjectMacro(ResliceCursorWidget,vtkResliceCursorWidget);
  //@}

  /**
   * Set/get the slice orientation
   */

  enum
  {
    RESLICE_AXIS_ALIGNED = 0,
    RESLICE_OBLIQUE = 1
  };

  vtkGetMacro(ResliceMode, int);
  virtual void SetResliceMode(int resliceMode);
  virtual void SetResliceModeToAxisAligned()
    { this->SetResliceMode(usResliceImageViewer::RESLICE_AXIS_ALIGNED); };
  virtual void SetResliceModeToOblique()
    { this->SetResliceMode(usResliceImageViewer::RESLICE_OBLIQUE); };

  //@{
  /**
   * Set/Get the reslice cursor.
   */
  vtkResliceCursor * GetResliceCursor();
  void SetResliceCursor( vtkResliceCursor * rc );
  //@}

  //@{
  /**
   * Set the lookup table
   */
  virtual void SetLookupTable( vtkScalarsToColors * );
  vtkScalarsToColors * GetLookupTable();
  //@}

  //@{
  /**
   * Switch to / from thick mode
   */
  virtual void SetThickMode( int );
  virtual int GetThickMode();
  //@}

  /**
   * Reset all views back to initial state
   */
  virtual void Reset();

  //@{
  /**
   * Get the point placer.
   */
  vtkGetObjectMacro( PointPlacer, vtkBoundedPlanePointPlacer );
  //@}

  //@{
  /**
   * Get the measurements manager
   */
  vtkGetObjectMacro( Measurements, vtkResliceImageViewerMeasurements );
  //@}

  //@{
  /**
   * Get the render window interactor
   */
  vtkGetObjectMacro( Interactor, vtkRenderWindowInteractor );
  //@}

  //@{
  /**
   * Scroll slices on the mouse wheel ? In the case of MPR
   * view, it moves one "normalized spacing" in the direction of the normal to
   * the resliced plane, provided the new center will continue to lie within
   * the volume.
   */
  vtkSetMacro( SliceScrollOnMouseWheel, int );
  vtkGetMacro( SliceScrollOnMouseWheel, int );
  vtkBooleanMacro( SliceScrollOnMouseWheel, int );
  //@}

  /**
   * Increment/Decrement slice by 'n' slices
   */
  virtual void IncrementSlice( int n );

  enum { SliceChangedEvent = 1001 };

protected:
  usResliceImageViewer();
  ~usResliceImageViewer();

  virtual void InstallPipeline();
  virtual void UnInstallPipeline();
  virtual void UpdateOrientation();
  virtual void UpdateDisplayExtent();
  virtual void UpdatePointPlacer();

  //@{
  /**
   * Convenience methods to get the reslice plane and the normalized
   * spacing between slices in reslice mode.
   */
  vtkPlane * GetReslicePlane();
  double GetInterSliceSpacingInResliceMode();
  //@}

  vtkResliceCursorWidget            * ResliceCursorWidget;
  vtkBoundedPlanePointPlacer        * PointPlacer;
  int                                 ResliceMode;
  vtkResliceImageViewerMeasurements * Measurements;
  int                                 SliceScrollOnMouseWheel;
  vtkResliceImageViewerScrollCallback * ScrollCallback;

private:
  usResliceImageViewer(const usResliceImageViewer&);
  void operator=(const usResliceImageViewer&);
};

#endif
