#ifndef US_MEDICAL_IMAGE_VIEWER
#define US_MEDICAL_IMAGE_VIEWER

//VISP includes
#include <visp3/core/vpConfig.h>

//USTK includes


//VTK includes
#include <vtkSmartPointer.h>
#include <vtkResliceImageViewer.h>
#include <vtkImagePlaneWidget.h>
#include <vtkDistanceWidget.h>
#include <vtkResliceImageViewerMeasurements.h>

//Qt includes
#include <QApplication>
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>

#include <QVTKWidget.h>

class VISP_EXPORT usMedicalImageViewer : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  usMedicalImageViewer(std::string imageFileName );
  ~usMedicalImageViewer() {}

  void resizeEvent(QResizeEvent* event);

public slots:

  virtual void ResetViews();
  virtual void Render();
  virtual void AddDistanceMeasurementToView1();
  virtual void AddDistanceMeasurementToView( int );
  virtual void slotExit();

protected:
  vtkSmartPointer< vtkResliceImageViewer > riw[3];
  vtkSmartPointer< vtkImagePlaneWidget > planeWidget[3];
  vtkSmartPointer< vtkDistanceWidget > DistanceWidget[3];
  vtkSmartPointer< vtkResliceImageViewerMeasurements > ResliceMeasurements[3];

protected slots:

private:
    void setupUi();

    QAction *actionOpenFile;
    QAction *actionExit;
    QAction *actionPrint;
    QAction *actionHelp;
    QAction *actionSave;
    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    QVTKWidget *view2;
    QVTKWidget *view4;
    QVTKWidget *view3;
    QVTKWidget *view1;
    QPushButton *resetButton;
    QPushButton *AddDistance1Button;
};

#endif // US_MEDICAL_IMAGE_VIEWER
